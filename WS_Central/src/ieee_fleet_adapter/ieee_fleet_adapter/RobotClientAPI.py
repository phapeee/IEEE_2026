import json
import math
import threading
from dataclasses import dataclass
from typing import Dict, Optional

from rclpy.action import ActionClient
from rclpy.qos import qos_profile_system_default
from rclpy.time import Time

from ieee_fleet_msgs.action import ExecuteCommand
from ieee_fleet_msgs.msg import RobotState


@dataclass
class RobotUpdateData:
    stamp: Time
    robot_name: str
    map_name: str
    position: list
    battery_soc: float
    battery_soc_valid: bool
    last_request_completed: int
    requires_replan: bool

    def is_command_completed(self, cmd_id: int) -> bool:
        return self.last_request_completed == cmd_id


class RobotAPI:
    def __init__(
        self,
        node,
        action_name: str,
        state_topic: str,
        command_timeout: float = 5.0,
        server_wait_timeout: float = 2.0,
        use_robot_namespace: bool = True,
        debug: bool = False,
    ):
        self._node = node
        self._action_name = action_name.strip("/")
        self._command_timeout = command_timeout
        self._server_wait_timeout = server_wait_timeout
        self._use_robot_namespace = use_robot_namespace
        self._debug = debug
        self._clients: Dict[str, ActionClient] = {}
        self._client_action_names: Dict[str, str] = {}
        self._goal_handles = {}
        self._state_lock = threading.Lock()
        self._states: Dict[str, RobotState] = {}

        self._state_sub = node.create_subscription(
            RobotState,
            state_topic,
            self._state_cb,
            qos_profile=qos_profile_system_default,
        )

    def _state_cb(self, msg: RobotState):
        if not msg.robot_name:
            self._node.get_logger().warn('RobotState missing robot_name; ignoring')
            return
        with self._state_lock:
            self._states[msg.robot_name] = msg

    def _client_for(self, robot_name: str) -> ActionClient:
        client = self._clients.get(robot_name)
        if client is not None:
            return client
        clean_name = robot_name.strip("/")
        if self._use_robot_namespace:
            action_full_name = f'/{clean_name}/{self._action_name}'
        else:
            action_full_name = f'/{self._action_name}'
        client = ActionClient(self._node, ExecuteCommand, action_full_name)
        self._clients[robot_name] = client
        self._client_action_names[robot_name] = action_full_name
        return client

    def check_connection(self) -> bool:
        with self._state_lock:
            return len(self._states) > 0

    def get_data(self, robot_name: Optional[str] = None):
        with self._state_lock:
            if robot_name is None:
                return [self._to_update_data(s) for s in self._states.values()]
            state = self._states.get(robot_name)
        if state is None:
            return None
        return self._to_update_data(state)

    def _to_update_data(self, state: RobotState) -> RobotUpdateData:
        battery_soc = float(state.battery_soc)
        battery_valid = math.isfinite(battery_soc) and 0.0 <= battery_soc <= 1.0
        return RobotUpdateData(
            stamp=Time.from_msg(state.stamp),
            robot_name=state.robot_name,
            map_name=state.map_name,
            position=[state.x, state.y, state.yaw],
            battery_soc=battery_soc,
            battery_soc_valid=battery_valid,
            last_request_completed=state.last_completed_request,
            requires_replan=state.requires_replan,
        )

    def send_command(self, robot_name: str, cmd_id: int, category: str, payload: dict) -> bool:
        client = self._client_for(robot_name)
        if not client.wait_for_server(timeout_sec=self._server_wait_timeout):
            if self._debug:
                action_name = self._client_action_names.get(robot_name, "?")
                self._node.get_logger().warn(
                    f'Action server for [{robot_name}] not available on '
                    f'[{action_name}]')
            return False

        goal = ExecuteCommand.Goal()
        goal.command_id = cmd_id
        goal.category = category
        goal.description_json = json.dumps(payload)

        accepted_event = threading.Event()
        accepted = False

        def _goal_done(fut):
            nonlocal accepted
            try:
                goal_handle = fut.result()
            except Exception as exc:  # pragma: no cover - defensive
                self._node.get_logger().error(
                    f'Failed to send goal to [{robot_name}]: {exc}')
                accepted_event.set()
                return
            accepted = goal_handle.accepted
            if accepted:
                self._goal_handles[robot_name] = goal_handle
            accepted_event.set()

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(_goal_done)

        if not accepted_event.wait(self._command_timeout):
            if self._debug:
                self._node.get_logger().warn(
                    f'Command timeout waiting for [{robot_name}] to accept goal')
            return False
        return accepted

    def stop(self, robot_name: str) -> bool:
        goal_handle = self._goal_handles.get(robot_name)
        if goal_handle is None:
            return True

        cancel_event = threading.Event()
        success = False

        def _cancel_done(fut):
            nonlocal success
            try:
                response = fut.result()
                success = len(response.goals_canceling) > 0
            except Exception as exc:  # pragma: no cover - defensive
                self._node.get_logger().error(
                    f'Failed to cancel goal for [{robot_name}]: {exc}')
            cancel_event.set()

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(_cancel_done)
        cancel_event.wait(self._command_timeout)
        return success

    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        pose,
        map_name: str,
        speed_limit: float = 0.0,
        dock: Optional[str] = None,
    ) -> bool:
        payload = {
            'map_name': map_name,
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
            'speed_limit': speed_limit,
        }
        if dock is not None:
            payload['dock'] = dock
            category = 'dock'
        else:
            category = 'navigate'
        return self.send_command(robot_name, cmd_id, category, payload)

    def perform_action(
        self,
        robot_name: str,
        cmd_id: int,
        category: str,
        description: dict,
    ) -> bool:
        payload = {
            'category': category,
            'description': description,
        }
        return self.send_command(robot_name, cmd_id, category, payload)
