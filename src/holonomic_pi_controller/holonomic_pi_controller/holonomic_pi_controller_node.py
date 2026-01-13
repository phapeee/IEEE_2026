import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import Float32
import tf2_ros
from tf2_ros import TransformException


def quaternion_to_yaw(q) -> float:
    """Convert quaternion to yaw (rotation about Z)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HolonomicPiController(Node):
    """Simple holonomic PI controller that consumes NavigateToPose goals and outputs cmd_vel."""

    def __init__(self) -> None:
        super().__init__("holonomic_pi_controller")

        self._cb_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._control_period = 1.0 / max(self._control_rate_hz, 1e-3)

        self._cmd_vel_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 10)
        unstamped_topic = self._cmd_vel_unstamped_topic.strip()
        self._cmd_vel_unstamped_pub = (
            self.create_publisher(Twist, unstamped_topic, 10) if unstamped_topic else None
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        self._goal_lock = threading.Lock()
        self._active_goal_handle = None
        self._active_stop_event: Optional[threading.Event] = None
        self._last_cmd = Twist()
        self._reset_integrators()
        self._next_tf_warning = 0.0
        self._error_pub = (
            self.create_publisher(Vector3Stamped, self._debug_error_topic, 10)
            if self._debug_enabled and self._debug_error_topic
            else None
        )
        self._distance_pub = (
            self.create_publisher(Float32, self._debug_distance_topic, 10)
            if self._debug_enabled and self._debug_distance_topic
            else None
        )

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            self._action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=self._handle_accepted,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"Holonomic PI controller ready on action '{self._action_name}', publishing to '{self._cmd_vel_topic}'"
        )

    def _declare_parameters(self) -> None:
        self._action_name = self.declare_parameter("action_name", "navigate_to_pose").value
        self._cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self._cmd_vel_unstamped_topic = self.declare_parameter("unstamped_cmd_vel_topic", "").value
        self._cmd_vel_frame_id = self.declare_parameter("cmd_vel_frame_id", "base_link").value
        self._global_frame = self.declare_parameter("global_frame", "map").value
        self._base_frame = self.declare_parameter("base_frame", "base_link").value
        self._debug_enabled = bool(self.declare_parameter("debug_enabled", False).value)
        self._debug_error_topic = self.declare_parameter("debug_error_topic", "").value
        self._debug_distance_topic = self.declare_parameter("debug_distance_topic", "").value
        self._control_rate_hz = float(self.declare_parameter("control_rate_hz", 30.0).value)
        self._position_tolerance = float(self.declare_parameter("position_tolerance", 0.05).value)
        self._yaw_tolerance = float(self.declare_parameter("yaw_tolerance", 0.05).value)
        self._slowdown_distance = float(self.declare_parameter("slowdown_distance", 0.3).value)
        self._yaw_slowdown_threshold = float(self.declare_parameter("yaw_slowdown_threshold", 0.5).value)
        self._kp_xy = float(self.declare_parameter("kp_xy", 1.5).value)
        self._ki_xy = float(self.declare_parameter("ki_xy", 0.0).value)
        self._kp_yaw = float(self.declare_parameter("kp_yaw", 2.0).value)
        self._ki_yaw = float(self.declare_parameter("ki_yaw", 0.0).value)
        self._integrator_limit_xy = float(self.declare_parameter("integrator_limit_xy", 0.3).value)
        self._integrator_limit_yaw = float(self.declare_parameter("integrator_limit_yaw", 0.5).value)
        self._max_linear_speed = float(self.declare_parameter("max_linear_speed", 0.6).value)
        self._min_linear_speed = float(self.declare_parameter("min_linear_speed", 0.0).value)
        self._max_angular_speed = float(self.declare_parameter("max_angular_speed", 1.5).value)
        self._min_angular_speed = float(self.declare_parameter("min_angular_speed", 0.0).value)
        self._max_linear_accel = float(self.declare_parameter("max_linear_accel", 0.6).value)
        self._max_linear_decel = float(self.declare_parameter("max_linear_decel", 0.8).value)
        self._max_angular_accel = float(self.declare_parameter("max_angular_accel", 2.0).value)
        self._max_angular_decel = float(self.declare_parameter("max_angular_decel", 2.5).value)
        self._goal_timeout_sec = float(self.declare_parameter("goal_timeout_sec", 120.0).value)
        self._transform_timeout_sec = float(self.declare_parameter("transform_timeout_sec", 1.0).value)

    def _goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(
            f"Received goal to ({goal_request.pose.pose.position.x:.3f}, "
            f"{goal_request.pose.pose.position.y:.3f}) in frame "
            f"'{goal_request.pose.header.frame_id or self._global_frame}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for active goal")
        with self._goal_lock:
            if self._active_goal_handle == goal_handle and self._active_stop_event:
                self._active_stop_event.set()
        return CancelResponse.ACCEPT

    def _handle_accepted(self, goal_handle) -> None:
        new_stop_event = threading.Event()
        with self._goal_lock:
            if self._active_goal_handle and self._active_goal_handle.is_active:
                self.get_logger().warn("Preempting active goal with a new request")
                if self._active_stop_event:
                    self._active_stop_event.set()
            self._active_goal_handle = goal_handle
            self._active_stop_event = new_stop_event
            self._reset_integrators()
            self._last_cmd = Twist()
        goal_handle.execute()

    def _execute_goal(self, goal_handle) -> NavigateToPose.Result:
        goal_pose = goal_handle.request.pose
        goal_pose.header.frame_id = goal_pose.header.frame_id or self._global_frame
        goal_yaw = quaternion_to_yaw(goal_pose.pose.orientation)
        with self._goal_lock:
            stop_event = self._active_stop_event or threading.Event()

        start_time = self.get_clock().now()
        last_update = time.monotonic()
        tf_failure_start: Optional[float] = None
        result = NavigateToPose.Result()
        result.result = Empty()

        while rclpy.ok():
            loop_start = time.monotonic()
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled by client")
                goal_handle.canceled()
                self._publish_stop()
                self._clear_active_goal(goal_handle)
                return result

            if stop_event.is_set():
                self.get_logger().warn("Goal aborted: preempted or canceled")
                goal_handle.abort()
                self._publish_stop()
                self._clear_active_goal(goal_handle)
                return result

            now = self.get_clock().now()
            dt = max(loop_start - last_update, 1e-3)
            last_update = loop_start

            current_pose = self._lookup_current_pose()
            if current_pose is None:
                if tf_failure_start is None:
                    tf_failure_start = loop_start
                elif (loop_start - tf_failure_start) > self._transform_timeout_sec:
                    self.get_logger().error("TF lookup timeout; aborting goal")
                    goal_handle.abort()
                    self._publish_stop()
                    self._clear_active_goal(goal_handle)
                    return result
                time.sleep(self._control_period)
                continue

            tf_failure_start = None
            errors = self._compute_errors(current_pose, goal_pose, goal_yaw)
            error_x, error_y, yaw_error, distance = errors
            self._publish_debug_errors(error_x, error_y, yaw_error, distance)

            elapsed = now - start_time
            self._publish_feedback(goal_handle, current_pose, elapsed, distance, yaw_error)

            if distance <= self._position_tolerance and abs(yaw_error) <= self._yaw_tolerance:
                self.get_logger().info("Goal reached within tolerance")
                self._publish_stop()
                goal_handle.succeed()
                self._clear_active_goal(goal_handle)
                return result

            if self._goal_timeout_sec > 0.0 and elapsed.nanoseconds * 1e-9 > self._goal_timeout_sec:
                timeout_sec = elapsed.nanoseconds * 1e-9
                self.get_logger().warn(f"Goal timeout reached ({timeout_sec:.1f} s)")
                goal_handle.abort()
                self._publish_stop()
                self._clear_active_goal(goal_handle)
                return result

            cmd = self._compute_control(error_x, error_y, yaw_error, distance, dt)
            vx, vy, wz = cmd
            self._publish_cmd(vx, vy, wz)

            sleep_time = self._control_period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._clear_active_goal(goal_handle)
        return result

        with self._goal_lock:
            if self._active_goal_handle == goal_handle:
                self._active_goal_handle = None
        return result

    def _lookup_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException as ex:
            now = time.monotonic()
            if now >= self._next_tf_warning:
                self.get_logger().warn(
                    f"Transform {self._global_frame}->{self._base_frame} unavailable: {ex}"
                )
                self._next_tf_warning = now + 1.0
            return None

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self._global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _compute_errors(
        self, current_pose: PoseStamped, goal_pose: PoseStamped, goal_yaw: float
    ) -> Tuple[float, float, float, float]:
        dx = goal_pose.pose.position.x - current_pose.pose.position.x
        dy = goal_pose.pose.position.y - current_pose.pose.position.y
        current_yaw = quaternion_to_yaw(current_pose.pose.orientation)
        yaw_error = normalize_angle(goal_yaw - current_yaw)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        # Rotate world-frame error into base frame (world -> body uses -yaw).
        error_x_base = cos_yaw * dx + sin_yaw * dy
        error_y_base = -sin_yaw * dx + cos_yaw * dy
        distance = math.hypot(dx, dy)
        return error_x_base, error_y_base, yaw_error, distance

    def _compute_control(
        self, error_x: float, error_y: float, yaw_error: float, distance: float, dt: float
    ) -> Tuple[float, float, float]:
        self._integral_x = self._clamp(
            self._integral_x + error_x * dt, -self._integrator_limit_xy, self._integrator_limit_xy
        )
        self._integral_y = self._clamp(
            self._integral_y + error_y * dt, -self._integrator_limit_xy, self._integrator_limit_xy
        )
        self._integral_yaw = self._clamp(
            self._integral_yaw + yaw_error * dt, -self._integrator_limit_yaw, self._integrator_limit_yaw
        )

        vx_raw = self._kp_xy * error_x + self._ki_xy * self._integral_x
        vy_raw = self._kp_xy * error_y + self._ki_xy * self._integral_y
        wz_raw = self._kp_yaw * yaw_error + self._ki_yaw * self._integral_yaw

        if self._slowdown_distance > 0.0 and distance < self._slowdown_distance:
            scale = max(distance / self._slowdown_distance, 0.05)
            vx_raw *= scale
            vy_raw *= scale

        if self._yaw_slowdown_threshold > 0.0 and abs(yaw_error) < self._yaw_slowdown_threshold:
            yaw_scale = max(abs(yaw_error) / self._yaw_slowdown_threshold, 0.05)
            wz_raw *= yaw_scale

        vx_raw, vy_raw = self._limit_planar_speed(vx_raw, vy_raw, distance)
        wz_raw = self._limit_value(wz_raw, self._min_angular_speed, self._max_angular_speed, abs(yaw_error))

        vx_cmd = self._limit_rate(vx_raw, self._last_cmd.linear.x, self._max_linear_accel, self._max_linear_decel, dt)
        vy_cmd = self._limit_rate(vy_raw, self._last_cmd.linear.y, self._max_linear_accel, self._max_linear_decel, dt)
        wz_cmd = self._limit_rate(wz_raw, self._last_cmd.angular.z, self._max_angular_accel, self._max_angular_decel, dt)

        vx_cmd, vy_cmd = self._limit_planar_speed(vx_cmd, vy_cmd, distance)
        wz_cmd = self._clamp_abs(
            wz_cmd,
            self._min_angular_speed if abs(yaw_error) > self._yaw_tolerance else 0.0,
            self._max_angular_speed,
        )

        self._last_cmd.linear.x = vx_cmd
        self._last_cmd.linear.y = vy_cmd
        self._last_cmd.angular.z = wz_cmd

        return vx_cmd, vy_cmd, wz_cmd

    def _limit_planar_speed(self, vx: float, vy: float, distance: float) -> Tuple[float, float]:
        speed = math.hypot(vx, vy)
        if speed > 0.0 and self._max_linear_speed > 0.0 and speed > self._max_linear_speed:
            scale = self._max_linear_speed / speed
            vx *= scale
            vy *= scale

        min_speed = self._min_linear_speed if distance > self._position_tolerance else 0.0
        if speed > 0.0 and min_speed > 0.0 and speed < min_speed:
            scale = min_speed / speed
            vx *= scale
            vy *= scale
        return vx, vy

    def _limit_value(self, value: float, min_mag: float, max_mag: float, error_mag: float) -> float:
        limited = value
        if max_mag > 0.0:
            limited = self._clamp_abs(limited, 0.0, max_mag)
        if min_mag > 0.0 and abs(limited) < min_mag and error_mag > self._yaw_tolerance:
            limited = math.copysign(min_mag, limited if limited != 0.0 else error_mag)
        return limited

    def _limit_rate(self, target: float, current: float, accel: float, decel: float, dt: float) -> float:
        if dt <= 0.0:
            return target
        delta = target - current
        increasing = (abs(target) > abs(current)) and (target * current >= 0.0)
        max_delta = accel * dt if increasing else decel * dt
        max_delta = max(max_delta, 0.0)
        delta = self._clamp(delta, -max_delta, max_delta)
        return current + delta

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._cmd_vel_frame_id
        stamped.twist = msg
        self._cmd_vel_pub.publish(stamped)
        if self._cmd_vel_unstamped_pub:
            self._cmd_vel_unstamped_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._last_cmd = Twist()
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._cmd_vel_frame_id
        stamped.twist = Twist()
        self._cmd_vel_pub.publish(stamped)
        if self._cmd_vel_unstamped_pub:
            self._cmd_vel_unstamped_pub.publish(Twist())

    def _publish_feedback(
        self, goal_handle, current_pose: PoseStamped, elapsed: Duration, distance: float, yaw_error: float
    ) -> None:
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = current_pose
        feedback.navigation_time = elapsed.to_msg()
        feedback.distance_remaining = float(distance)
        feedback.number_of_recoveries = 0

        if self._max_linear_speed > 0.0 or self._max_angular_speed > 0.0:
            linear_time = distance / self._max_linear_speed if self._max_linear_speed > 0.0 else 0.0
            angular_time = abs(yaw_error) / self._max_angular_speed if self._max_angular_speed > 0.0 else 0.0
            est_time = max(linear_time, angular_time)
        else:
            est_time = 0.0
        feedback.estimated_time_remaining = Duration(seconds=est_time).to_msg()

        goal_handle.publish_feedback(feedback)

    def _reset_integrators(self) -> None:
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._integral_yaw = 0.0

    def _publish_debug_errors(self, error_x: float, error_y: float, yaw_error: float, distance: float) -> None:
        if self._error_pub:
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._base_frame
            msg.vector.x = error_x
            msg.vector.y = error_y
            msg.vector.z = yaw_error
            self._error_pub.publish(msg)
        if self._distance_pub:
            dist_msg = Float32()
            dist_msg.data = float(distance)
            self._distance_pub.publish(dist_msg)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return min(max(value, low), high)

    @staticmethod
    def _clamp_abs(value: float, min_mag: float, max_mag: float) -> float:
        if max_mag > 0.0 and abs(value) > max_mag:
            value = math.copysign(max_mag, value)
        if min_mag > 0.0 and abs(value) < min_mag:
            value = math.copysign(min_mag, value)
        return value

    def _clear_active_goal(self, goal_handle) -> None:
        with self._goal_lock:
            if self._active_goal_handle == goal_handle:
                self._active_goal_handle = None
                self._active_stop_event = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HolonomicPiController()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
