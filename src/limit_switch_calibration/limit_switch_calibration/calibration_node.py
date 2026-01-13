import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from robot_localization.srv import SetPose
from smacc2_msgs.msg import SmaccEvent
from std_msgs.msg import Bool, String
import yaml


class StateMachineError(Exception):
    """Generic error while running a state machine."""


class StateMachineComplete(Exception):
    """Raised when a state machine finishes via a terminate action."""

    def __init__(self, success: bool, message: str = "") -> None:
        super().__init__(message)
        self.success = success
        self.message = message


class LimitSwitchCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("limit_switch_calibration")
        self._callback_group = ReentrantCallbackGroup()

        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.cmd_vel_frame_id = self.declare_parameter("cmd_vel_frame_id", "base_link").value
        self.start_topic = self.declare_parameter("start_topic", "limit_switch_calibration/start").value
        self.status_topic = self.declare_parameter("status_topic", "limit_switch_calibration/status").value
        self.smacc_event_topic = self.declare_parameter("smacc_event_topic", "smacc2/calibration_events").value
        self.event_object_tag = self.declare_parameter("event_object_tag", "LimitSwitchCalibration").value
        self.event_source = self.declare_parameter("event_source", "limit_switch_calibration_node").value
        self.success_event_type = self.declare_parameter("success_event_type", "CALIBRATION_SUCCESS").value
        self.failure_event_type = self.declare_parameter("failure_event_type", "CALIBRATION_FAILURE").value
        self.set_pose_service_name = self.declare_parameter("set_pose_service", "/set_pose").value
        self.move_publish_rate_hz = float(self.declare_parameter("move_publish_rate_hz", 20.0).value)
        self.default_timeout_sec = float(self.declare_parameter("default_timeout_sec", 10.0).value)

        self.state_machine_file = str(self.declare_parameter("state_machine_file", "").value)
        self.switch_topics: Dict[str, str] = {
            "front": str(self.declare_parameter("front_switch_topic", "smacc2/button_state").value),
            "back": str(self.declare_parameter("back_switch_topic", "smacc2/back_swtich_state").value),
            "left": str(self.declare_parameter("left_switch_topic", "smacc2/left_switch_state").value),
            "right": str(self.declare_parameter("right_switch_topic", "smacc2/right_switch_state").value),
        }

        raw_state_machines = self._load_state_machines_from_file(self.state_machine_file)
        self.state_machines = self._normalize_state_machines(raw_state_machines)
        if not self.state_machines:
            self.get_logger().warn("No state machines defined. Publish a start command only after configuring them.")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, qos)
        self.status_pub = self.create_publisher(String, self.status_topic, qos)
        self.event_pub = self.create_publisher(SmaccEvent, self.smacc_event_topic, qos)

        self.create_subscription(String, self.start_topic, self._handle_start_request, qos)

        self._switch_states: Dict[str, Optional[bool]] = {name: None for name in self.switch_topics}
        self._state_condition = threading.Condition()
        for name, topic in self.switch_topics.items():
            self.create_subscription(Bool, topic, self._make_switch_callback(name), qos)

        self._set_pose_client = self.create_client(SetPose, self.set_pose_service_name, callback_group=self._callback_group)

        self._worker_thread: Optional[threading.Thread] = None
        self._cancel_event = threading.Event()
        self._active_machine_id: Optional[str] = None
        self.get_logger().info(
            f"limit_switch_calibration node ready: start topic={self.start_topic} "
            f"status topic={self.status_topic} cmd_vel={self.cmd_vel_topic}"
        )

    def _normalize_state_machines(self, raw_param: Any) -> Dict[str, Dict[str, Any]]:
        if raw_param in (None, ""):
            return {}
        if not isinstance(raw_param, dict):
            raise ValueError("state_machines parameter must be a mapping of id -> definition")
        normalized: Dict[str, Dict[str, Any]] = {}
        for sm_id, definition in raw_param.items():
            if not isinstance(definition, dict):
                raise ValueError(f"State machine '{sm_id}' must be a dictionary")
            steps = definition.get("steps", [])
            if not isinstance(steps, list) or not steps:
                raise ValueError(f"State machine '{sm_id}' must contain a non-empty 'steps' list")
            normalized[sm_id] = {
                "description": definition.get("description", ""),
                "steps": steps,
            }
        return normalized

    def _load_state_machines_from_file(self, file_path: str) -> Dict[str, Any]:
        if not file_path:
            return {}
        path = Path(file_path)
        if not path.exists():
            self.get_logger().error(f"state_machine_file '{file_path}' does not exist")
            return {}
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to parse state_machine_file '{file_path}': {exc}")
            return {}
        machines = data.get("state_machines", {})
        if not isinstance(machines, dict):
            self.get_logger().error(
                f"state_machine_file '{file_path}' must contain a 'state_machines' mapping"
            )
            return {}
        self.get_logger().info(f"Loaded {len(machines)} state machine(s) from {file_path}")
        return machines

    def _make_switch_callback(self, name: str):
        def _callback(msg: Bool) -> None:
            pressed = bool(msg.data)
            with self._state_condition:
                previous = self._switch_states.get(name)
                self._switch_states[name] = pressed
                if previous != pressed:
                    self._state_condition.notify_all()
        return _callback

    def _handle_start_request(self, msg: String) -> None:
        machine_id = msg.data.strip()
        if not machine_id:
            self.get_logger().warn("Received empty state machine id")
            return
        if machine_id not in self.state_machines:
            self.get_logger().error(f"Unknown state machine id '{machine_id}'")
            return
        if self._worker_thread and self._worker_thread.is_alive():
            self.get_logger().warn(
                f"State machine '{machine_id}' requested but '{self._active_machine_id}' is still running"
            )
            return
        self.get_logger().info(f"Starting state machine '{machine_id}'")
        self._cancel_event = threading.Event()
        self._active_machine_id = machine_id
        self._worker_thread = threading.Thread(
            target=self._run_state_machine,
            args=(machine_id, self.state_machines[machine_id], self._cancel_event),
            daemon=True,
        )
        self._worker_thread.start()

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _emit_smacc_event(self, event_type: str, label: str) -> None:
        event = SmaccEvent()
        event.event_type = event_type
        event.event_object_tag = self.event_object_tag
        event.event_source = self.event_source
        event.label = label
        self.event_pub.publish(event)

    def _run_linear_state_machine(self, machine_id: str, definition: Dict[str, Any], cancel_event: threading.Event) -> None:
        for index, step in enumerate(definition.get("steps", []), start=1):
            step_type = step.get("type")
            if not isinstance(step_type, str):
                raise StateMachineError(f"Invalid step type in '{machine_id}' (step {index})")

            if step_type == "condition":
                self._execute_condition_step(step, cancel_event)
            elif step_type == "move":
                self._execute_move_step(step, cancel_event)
            elif step_type == "action":
                self._execute_action_step(step, cancel_event)
            else:
                raise StateMachineError(f"Unsupported step type '{step_type}' in '{machine_id}'")

            self._publish_status(f"State machine '{machine_id}' completed step {index}: {step_type}")

    def _run_graph_state_machine(self, machine_id: str, definition: Dict[str, Any], cancel_event: threading.Event) -> None:
        steps_list = definition.get("steps", [])
        if not steps_list:
            raise StateMachineError(f"State machine '{machine_id}' has no steps")

        # Build name -> step map
        step_map: Dict[str, Dict[str, Any]] = {}
        for step in steps_list:
            name = step.get("name")
            if not name:
                raise StateMachineError(f"All steps in graph-mode state machine '{machine_id}' must have a 'name'")
            if name in step_map:
                raise StateMachineError(f"Duplicate step name '{name}' in state machine '{machine_id}'")
            step_map[name] = step

        # Start state: explicit or first step
        current_state = definition.get("start_state") or steps_list[0].get("name")
        if current_state not in step_map:
            raise StateMachineError(f"Start state '{current_state}' not found in state machine '{machine_id}'")

        hop_count = 0
        max_hops = 1000  # simple protection against accidental infinite loops

        while True:
            if cancel_event.is_set():
                raise StateMachineError(f"State machine '{machine_id}' cancelled")

            step = step_map[current_state]
            step_type = step.get("type")
            if not isinstance(step_type, str):
                raise StateMachineError(f"Invalid step type in '{machine_id}' at state '{current_state}'")

            self._publish_status(f"Executing state '{current_state}' ({step_type})")

            # --- MOVE STATE ---
            if step_type == "move":
                result = self._execute_move_step_with_condition(step, cancel_event)
                if result == "success":
                    next_state = step.get("next_successive_state")
                elif result == "timeout":
                    next_state = step.get("next_timeout_state")
                else:
                    raise StateMachineError(f"Move state '{current_state}' returned unknown result '{result}'")

            # --- SET_POSE STATE (graph-style shorthand) ---
            elif step_type == "set_pose":
                try:
                    self._execute_set_pose_action(step, cancel_event)
                    # Service OK -> "set_pose_success" conceptual condition
                    next_state = step.get("next_successive_state")
                except StateMachineError as exc:
                    # Service error -> "set_pose_failure"
                    self.get_logger().warn(f"set_pose failed in state '{current_state}': {exc}")
                    next_state = step.get("next_failure_state")
                    if not next_state:
                        # If you didn't configure a failure branch, treat as fatal
                        raise

            # --- TERMINATE STATE ---
            elif step_type == "terminate":
                success = bool(step.get("success", True))
                message = step.get("message", f"{machine_id}:{current_state}")
                raise StateMachineComplete(success=success, message=message)

            # --- OPTIONAL: condition state with branch (simple version) ---
            elif step_type == "condition":
                # You *can* reuse the old condition step and then always go to next_successive_state
                self._execute_condition_step(step, cancel_event)
                next_state = step.get("next_successive_state")

            else:
                raise StateMachineError(f"Unsupported step type '{step_type}' in '{machine_id}' at state '{current_state}'")

            if not next_state:
                # No explicit next state -> treat as successful completion
                raise StateMachineComplete(success=True, message=f"{machine_id}:{current_state}:complete")

            if next_state not in step_map:
                raise StateMachineError(
                    f"Next state '{next_state}' from '{current_state}' not found in state machine '{machine_id}'"
                )

            self._publish_status(f"Transition: {current_state} -> {next_state}")
            current_state = next_state

            hop_count += 1
            if hop_count > max_hops:
                raise StateMachineError(f"Exceeded max transitions in '{machine_id}', possible infinite loop")

    def _execute_move_step_with_condition(self, step: Dict[str, Any], cancel_event: threading.Event) -> str:
        """
        Graph-mode move:
          - If success_condition / success_any_of / success_all_of is given:
              drive until condition == True or timeout_sec expires.
              return "success" or "timeout".
          - Else if duration_sec > 0:
              drive for duration_sec and return "success".
        """
        vx = float(step.get("velocity_x", 0.0))
        vy = float(step.get("velocity_y", 0.0))

        success_any_of = step.get("success_any_of")
        success_all_of = step.get("success_all_of")
        success_condition = step.get("success_condition")
        timeout_sec = float(step.get("timeout_sec", 0.0))
        duration_sec = float(step.get("duration_sec", 0.0))

        # Build condition target list
        targets: List[str] = []
        mode = "single"

        if isinstance(success_any_of, list) and success_any_of:
            targets = [str(item) for item in success_any_of]
            mode = "any"
        elif isinstance(success_all_of, list) and success_all_of:
            targets = [str(item) for item in success_all_of]
            mode = "all"
        elif isinstance(success_condition, str):
            targets = [success_condition]

        # No more raw Twist here – we use the helper
        period = 1.0 / max(1.0, self.move_publish_rate_hz)

        # For condition-based moves, use timeout_sec as deadline
        if targets:
            deadline = time.monotonic() + timeout_sec if timeout_sec > 0.0 else None
            try:
                while True:
                    if cancel_event.is_set():
                        raise StateMachineError("Move step cancelled")

                    if self._conditions_met(targets, mode):
                        return "success"

                    if deadline is not None and time.monotonic() >= deadline:
                        return "timeout"

                    self._publish_cmd_vel(vx, vy)
                    time.sleep(period)
            finally:
                self._send_zero_twist()

        # No condition => duration-based move (backward-compatible semantics)
        if duration_sec <= 0.0:
            raise StateMachineError("Graph-mode move requires either success_condition or duration_sec > 0")

        end_time = time.monotonic() + duration_sec
        try:
            while time.monotonic() < end_time:
                if cancel_event.is_set():
                    raise StateMachineError("Move step cancelled")
                self._publish_cmd_vel(vx, vy)
                time.sleep(period)
        finally:
            self._send_zero_twist()

        return "success"

    def _publish_cmd_vel(self, vx: float, vy: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_vel_frame_id
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        # angular.z (yaw) stays 0 for now, but you can extend this later
        self.cmd_vel_pub.publish(msg)

    def _run_state_machine(self, machine_id: str, definition: Dict[str, Any], cancel_event: threading.Event) -> None:
        description = definition.get("description", "")
        self._publish_status(f"State machine '{machine_id}' started. {description}")
        try:
            steps = definition.get("steps", [])
            use_graph_mode = any(isinstance(s, dict) and "name" in s for s in steps) or "start_state" in definition

            if use_graph_mode:
                self._run_graph_state_machine(machine_id, definition, cancel_event)
            else:
                self._run_linear_state_machine(machine_id, definition, cancel_event)

            # Completed the entire plan without an explicit terminate action
            self._emit_smacc_event(self.success_event_type, f"{machine_id}:complete")
            self._publish_status(f"State machine '{machine_id}' finished successfully")

        except StateMachineComplete as result:
            event_type = self.success_event_type if result.success else self.failure_event_type
            label = result.message or (f"{machine_id}:{'success' if result.success else 'failure'}")
            self._emit_smacc_event(event_type, label)
            self._publish_status(f"State machine '{machine_id}' terminated: {label}")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"State machine '{machine_id}' failed: {exc}")
            self._emit_smacc_event(self.failure_event_type, f"{machine_id}:error")
            self._publish_status(f"State machine '{machine_id}' failed: {exc}")
        finally:
            self._send_zero_twist()
            self._active_machine_id = None

    def _execute_condition_step(self, step: Dict[str, Any], cancel_event: threading.Event) -> None:
        timeout_sec = float(step.get("timeout_sec", self.default_timeout_sec))
        any_of = step.get("any_of")
        all_of = step.get("all_of")
        condition = step.get("condition")
        if condition == "timer":
            duration = float(step.get("duration_sec", timeout_sec))
            self._wait_with_cancel(duration, cancel_event)
            return

        targets: List[str]
        mode = "single"
        if isinstance(any_of, list) and any_of:
            targets = [str(item) for item in any_of]
            mode = "any"
        elif isinstance(all_of, list) and all_of:
            targets = [str(item) for item in all_of]
            mode = "all"
        elif isinstance(condition, str):
            targets = [condition]
        else:
            raise StateMachineError("Condition step must define 'condition', 'any_of', or 'all_of'")

        deadline = time.monotonic() + timeout_sec if timeout_sec > 0 else None
        with self._state_condition:
            while not self._conditions_met(targets, mode):
                if cancel_event.is_set():
                    raise StateMachineError("Condition wait cancelled")
                if deadline:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise StateMachineError("Condition step timed out")
                    self._state_condition.wait(timeout=remaining)
                else:
                    self._state_condition.wait(timeout=0.1)

    def _wait_with_cancel(self, duration: float, cancel_event: threading.Event) -> None:
        end_time = time.monotonic() + max(0.0, duration)
        while time.monotonic() < end_time:
            if cancel_event.is_set():
                raise StateMachineError("Timer wait cancelled")
            time.sleep(0.05)

    def _conditions_met(self, names: List[str], mode: str) -> bool:
        values = [self._evaluate_condition(name) for name in names]
        if mode == "all":
            return all(values)
        if mode == "any":
            return any(values)
        return values[0]

    def _evaluate_condition(self, name: str) -> bool:
        state = self._switch_states
        mapping = {
            "front_pressed": bool(state.get("front")),
            "front_released": not bool(state.get("front")),
            "back_pressed": bool(state.get("back")),
            "back_released": not bool(state.get("back")),
            "left_pressed": bool(state.get("left")),
            "left_released": not bool(state.get("left")),
            "right_pressed": bool(state.get("right")),
            "right_released": not bool(state.get("right")),
        }
        if name not in mapping:
            raise StateMachineError(f"Unknown condition '{name}'")
        return mapping[name]

    def _execute_move_step(self, step: Dict[str, Any], cancel_event: threading.Event) -> None:
        duration = float(step.get("duration_sec", 0.0))
        vx = float(step.get("velocity_x", 0.0))
        vy = float(step.get("velocity_y", 0.0))
        if duration <= 0.0:
            raise StateMachineError("Move step requires a positive duration_sec")

        period = 1.0 / max(1.0, self.move_publish_rate_hz)
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            if cancel_event.is_set():
                raise StateMachineError("Move step cancelled")
            self._publish_cmd_vel(vx, vy)
            time.sleep(period)
        self._send_zero_twist()

    def _execute_action_step(self, step: Dict[str, Any], cancel_event: threading.Event) -> None:
        action = step.get("action")
        if action == "set_pose":
            self._execute_set_pose_action(step, cancel_event)
        elif action == "terminate":
            result = step.get("result", "success").lower()
            message = step.get("message", "")
            success = result != "failure"
            raise StateMachineComplete(success=success, message=message)
        else:
            raise StateMachineError(f"Unsupported action '{action}'")

    def _execute_set_pose_action(self, step: Dict[str, Any], cancel_event: threading.Event) -> None:
        pose_cfg = step.get("pose", {})
        if not isinstance(pose_cfg, dict):
            raise StateMachineError("set_pose action requires a 'pose' dictionary")
        frame_id = str(pose_cfg.get("frame_id", "map"))
        x = float(pose_cfg.get("x", 0.0))
        y = float(pose_cfg.get("y", 0.0))
        yaw = pose_cfg.get("yaw")
        if yaw is None:
            yaw_deg = float(pose_cfg.get("yaw_deg", 0.0))
            yaw = math.radians(yaw_deg)
        else:
            yaw = float(yaw)
        covariance = pose_cfg.get("covariance_diagonal", [0.01, 0.01, 0.01])
        if len(covariance) != 3:
            raise StateMachineError("covariance_diagonal must contain [x, y, yaw]")

        request = SetPose.Request()
        request.pose.header.frame_id = frame_id
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.position.x = x
        request.pose.pose.pose.position.y = y
        request.pose.pose.pose.position.z = 0.0
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0
        request.pose.pose.pose.orientation.z = qz
        request.pose.pose.pose.orientation.w = qw
        request.pose.pose.covariance = [0.0] * 36
        request.pose.pose.covariance[0] = float(covariance[0])
        request.pose.pose.covariance[7] = float(covariance[1])
        request.pose.pose.covariance[35] = float(covariance[2])

        timeout_sec = float(step.get("timeout_sec", self.default_timeout_sec))
        if not self._set_pose_client.wait_for_service(timeout_sec=timeout_sec):
            raise StateMachineError("set_pose service is unavailable")
        future = self._set_pose_client.call_async(request)
        start_time = time.monotonic()
        while not future.done():
            if cancel_event.is_set():
                raise StateMachineError("set_pose action cancelled")
            if timeout_sec > 0 and (time.monotonic() - start_time) > timeout_sec:
                raise StateMachineError("set_pose action timed out")
            time.sleep(0.05)
        if future.result() is None:
            raise StateMachineError("set_pose service failed")

    def _send_zero_twist(self) -> None:
        try:
            self._publish_cmd_vel(0.0, 0.0)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().debug(f"Failed to publish zero twist: {exc}")

    def destroy_node(self) -> bool:
        if self._worker_thread and self._worker_thread.is_alive():
            self._cancel_event.set()
            self._worker_thread.join(timeout=1.0)
        self._send_zero_twist()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LimitSwitchCalibrationNode()
    try:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            node.get_logger().info("External shutdown requested")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
