import argparse
import asyncio
from dataclasses import dataclass, field
import json
import math
import sys
import threading
import time
from typing import Dict, Optional, Set

import rclpy
from rclpy.duration import Duration
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy
from rmf_fleet_msgs.msg import ClosedLanes
from rmf_fleet_msgs.msg import LaneRequest
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import SpeedLimitRequest
from std_msgs.msg import String as StringMsg
import yaml

from .RobotClientAPI import RobotAPI
from .RobotClientAPI import RobotUpdateData


@dataclass
class AdapterConfig:
    state_stale_threshold_sec: float = 2.0
    battery_fallback_soc: float = 1.0
    low_battery_threshold: float = 0.2
    enable_battery_gating: bool = True
    critical_action_categories: Set[str] = field(default_factory=set)
    action_timeouts: Dict[str, float] = field(default_factory=dict)
    default_action_timeout_sec: float = 30.0
    navigate_timeout_base_sec: float = 5.0
    navigate_timeout_per_meter_sec: float = 3.0
    navigate_timeout_min_sec: float = 10.0
    navigate_timeout_max_sec: float = 300.0
    retry_backoff_sec: float = 1.0
    health_topic: str = '/ieee_fleet/adapter_health'
    health_publish_period_sec: float = 2.0
    metrics_publish_period_sec: float = 30.0
    nominal_linear_speed: float = 0.5
    allowed_actions: Set[str] = field(default_factory=set)
    required_action_keys: Dict[str, list] = field(default_factory=dict)


DEFAULT_REQUIRED_ACTION_KEYS = {
    'deploy': ['site'],
    'activate_antenna': ['antenna_id'],
    'pickup_duck': ['pickup_zone'],
    'drop_duck': ['drop_zone'],
    'crater_lap_leader': ['lap_count'],
    'crater_lap_follower': ['leader_id'],
    'dock': ['dock'],
    'takeoff': ['altitude_m'],
    'scan_leds': ['pattern'],
    'send_ir': ['payload'],
}


@dataclass
class ActiveCommand:
    cmd_id: int
    category: str
    start_time: float
    timeout_sec: float
    execution: object


class MetricsTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self._stats: Dict[str, Dict[str, float]] = {}
        self._replans = 0.0

    def record(self, category: str, outcome: str, duration_sec: float):
        with self._lock:
            entry = self._stats.setdefault(
                category,
                {
                    'success': 0.0,
                    'failure': 0.0,
                    'timeout': 0.0,
                    'preempted': 0.0,
                    'total_duration': 0.0,
                    'count_duration': 0.0,
                },
            )
            if outcome not in entry:
                entry[outcome] = 0.0
            entry[outcome] += 1.0
            if duration_sec is not None:
                entry['total_duration'] += duration_sec
                entry['count_duration'] += 1.0

    def record_replan(self):
        with self._lock:
            self._replans += 1.0

    def snapshot(self) -> Dict[str, Dict[str, float]]:
        with self._lock:
            snapshot = json.loads(json.dumps(self._stats))
            for category, entry in snapshot.items():
                count = entry.get('count_duration', 0.0)
                if count > 0:
                    entry['avg_duration'] = entry.get('total_duration', 0.0) / count
                else:
                    entry['avg_duration'] = 0.0
            snapshot['__replans__'] = {'count': self._replans}
            return snapshot


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------

def main(argv=sys.argv):
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Configure and spin up the IEEE 2026 fleet adapter',
    )
    parser.add_argument(
        '-c',
        '--config_file',
        type=str,
        required=True,
        help='Path to the config.yaml file',
    )
    parser.add_argument(
        '-n',
        '--nav_graph',
        type=str,
        required=True,
        help='Path to the nav_graph for this fleet adapter',
    )
    parser.add_argument(
        '-sim',
        '--use_sim_time',
        action='store_true',
        help='Use sim time, default: false',
    )
    args = parser.parse_args(args_without_ros[1:])
    print('Starting IEEE fleet adapter...')

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. '
        'Please ensure RMF Schedule Node is running'
    )

    if args.use_sim_time:
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    node.declare_parameter('server_uri', '')
    server_uri = (
        node.get_parameter('server_uri').get_parameter_value().string_value
    )
    if server_uri == '':
        server_uri = None

    fleet_config.server_uri = server_uri
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    # Parse adapter-specific settings
    fleet_mgr_yaml = config_yaml.get('fleet_manager', {})
    rmf_fleet_yaml = config_yaml.get('rmf_fleet', {})
    limits_yaml = rmf_fleet_yaml.get('limits', {})
    linear_limits = limits_yaml.get('linear', [0.5, 1.0])
    nominal_speed = 0.5
    if isinstance(linear_limits, (list, tuple)) and len(linear_limits) > 0:
        nominal_speed = float(linear_limits[0])

    update_frequency = fleet_mgr_yaml.get('robot_state_update_frequency', 10.0)
    update_period = 1.0 / max(update_frequency, 1e-3)

    adapter_config = AdapterConfig(
        state_stale_threshold_sec=fleet_mgr_yaml.get('state_stale_threshold_sec', 2.0),
        battery_fallback_soc=fleet_mgr_yaml.get('battery_fallback_soc', 1.0),
        low_battery_threshold=fleet_mgr_yaml.get('low_battery_threshold', 0.2),
        enable_battery_gating=fleet_mgr_yaml.get('enable_battery_gating', True),
        critical_action_categories=set(
            fleet_mgr_yaml.get('critical_action_categories', [])
        ),
        action_timeouts=fleet_mgr_yaml.get('action_timeouts', {}),
        default_action_timeout_sec=fleet_mgr_yaml.get('default_action_timeout_sec', 30.0),
        navigate_timeout_base_sec=fleet_mgr_yaml.get('navigate_timeout_base_sec', 5.0),
        navigate_timeout_per_meter_sec=fleet_mgr_yaml.get('navigate_timeout_per_meter_sec', 3.0),
        navigate_timeout_min_sec=fleet_mgr_yaml.get('navigate_timeout_min_sec', 10.0),
        navigate_timeout_max_sec=fleet_mgr_yaml.get('navigate_timeout_max_sec', 300.0),
        retry_backoff_sec=fleet_mgr_yaml.get('retry_backoff_sec', 1.0),
        health_topic=fleet_mgr_yaml.get('health_topic', '/ieee_fleet/adapter_health'),
        health_publish_period_sec=fleet_mgr_yaml.get('health_publish_period_sec', 2.0),
        metrics_publish_period_sec=fleet_mgr_yaml.get('metrics_publish_period_sec', 30.0),
        nominal_linear_speed=nominal_speed,
        allowed_actions=set(rmf_fleet_yaml.get('actions', [])),
        required_action_keys=fleet_mgr_yaml.get(
            'required_action_keys', DEFAULT_REQUIRED_ACTION_KEYS
        ),
    )
    if not adapter_config.allowed_actions:
        node.get_logger().warn(
            'No action categories configured; perform_action will be rejected'
        )

    api = RobotAPI(
        node=node,
        action_name=fleet_mgr_yaml.get('command_action', 'execute_command'),
        state_topic=fleet_mgr_yaml.get('robot_state_topic', '/ieee_fleet/robot_state'),
        command_timeout=fleet_mgr_yaml.get('command_timeout', 5.0),
        server_wait_timeout=fleet_mgr_yaml.get('action_server_wait_timeout', 2.0),
        use_robot_namespace=fleet_mgr_yaml.get('use_robot_namespace', True),
        debug=fleet_mgr_yaml.get('debug', False),
    )

    metrics = MetricsTracker()
    health_pub = node.create_publisher(StringMsg, adapter_config.health_topic, 10)

    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name, robot_config, node, api, fleet_handle, adapter_config, metrics
        )

    def update_loop():
        reassign_task_interval = config_yaml['rmf_fleet'].get(
            'reassign_task_interval', 60)  # seconds
        last_task_replan = node.get_clock().now()
        last_metrics_publish = time.time()
        last_health_publish = time.time()
        asyncio.set_event_loop(asyncio.new_event_loop())
        while rclpy.ok():
            now = node.get_clock().now()

            update_jobs = []
            for robot in robots.values():
                update_jobs.append(update_robot(robot))

            asyncio.get_event_loop().run_until_complete(
                asyncio.wait(update_jobs)
            )

            interval_sec = (now.nanoseconds -
                            last_task_replan.nanoseconds) / 1e9
            if interval_sec > reassign_task_interval:
                fleet_handle.more().reassign_dispatched_tasks()
                last_task_replan = now

            now_wall = time.time()
            if now_wall - last_health_publish >= adapter_config.health_publish_period_sec:
                health_payload = {}
                for name, robot in robots.items():
                    age = robot.last_state_age_sec(now)
                    health_payload[name] = {
                        'age_sec': age,
                        'stale': age is None or age > adapter_config.state_stale_threshold_sec,
                    }
                msg = StringMsg()
                msg.data = json.dumps(health_payload)
                health_pub.publish(msg)
                last_health_publish = now_wall

            if now_wall - last_metrics_publish >= adapter_config.metrics_publish_period_sec:
                snapshot = metrics.snapshot()
                node.get_logger().info(
                    f'Adapter metrics: {json.dumps(snapshot)}'
                )
                last_metrics_publish = now_wall

            next_wakeup = now + Duration(nanoseconds=update_period * 1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    connections = ros_connections(node, robots, fleet_handle)
    connections  # Avoid unused variable warning

    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    rclpy_executor.spin()

    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


class RobotAdapter:
    def __init__(
        self,
        name: str,
        configuration,
        node,
        api: RobotAPI,
        fleet_handle,
        adapter_config: AdapterConfig,
        metrics: MetricsTracker,
    ):
        self.name = name
        self.execution = None
        self.active_command: Optional[ActiveCommand] = None
        self.cmd_id = 0
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.adapter_config = adapter_config
        self.metrics = metrics
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()
        self._cmd_lock = threading.Lock()
        self.pending_cmd_id: Optional[int] = None
        self.pending_category: Optional[str] = None
        self.pending_started_at: Optional[float] = None
        self.last_state_time = None
        self.last_position = None
        self.last_map_name = None
        self.last_battery_soc = None
        self._decommission_reasons: Set[str] = set()
        self._battery_missing_logged = False
        self._last_requires_replan = False

    def update(self, data: RobotUpdateData):
        now = self.node.get_clock().now()
        self.last_state_time = data.stamp
        self.last_position = data.position
        if data.map_name:
            self.last_map_name = data.map_name
        elif self.last_map_name is None:
            self.last_map_name = 'unknown'
            self.node.get_logger().error(
                f'[{self.name}] RobotState missing map_name'
            )

        if self._is_state_stale(now):
            self._set_unavailable('stale')
        else:
            self._clear_unavailable('stale')

        battery_soc = data.battery_soc
        if not data.battery_soc_valid:
            if not self._battery_missing_logged:
                self.node.get_logger().warn(
                    f'[{self.name}] Battery SOC missing/invalid; '
                    f'using fallback {self.adapter_config.battery_fallback_soc}'
                )
                self._battery_missing_logged = True
            battery_soc = float(self.adapter_config.battery_fallback_soc)
        else:
            self._battery_missing_logged = False
        self.last_battery_soc = battery_soc

        if self.adapter_config.enable_battery_gating:
            if battery_soc < self.adapter_config.low_battery_threshold:
                self._set_unavailable('low_battery')
            else:
                self._clear_unavailable('low_battery')

        if data.requires_replan and not self._last_requires_replan:
            self.node.get_logger().warn(
                f'[{self.name}] requires_replan=true; triggering RMF replan'
            )
            self._request_replan()
            self.fleet_handle.more().reassign_dispatched_tasks()
            self.metrics.record_replan()
        self._last_requires_replan = data.requires_replan

        activity_identifier = None
        with self._cmd_lock:
            execution = self.execution
            active_command = self.active_command
        if execution:
            if data.is_command_completed(self.cmd_id):
                self._finish_active_command(success=True, reason='completed')
                with self._cmd_lock:
                    self.execution = None
                    self.active_command = None
            else:
                activity_identifier = execution.identifier

        if active_command is not None:
            self._check_timeout()

        map_name = self.last_map_name or data.map_name
        state = rmf_easy.RobotState(map_name, data.position, battery_soc)
        self.update_handle.update(state, activity_identifier)

    def last_state_age_sec(self, now_time):
        if self.last_state_time is None:
            return None
        age_ns = now_time.nanoseconds - self.last_state_time.nanoseconds
        return age_ns / 1e9

    def _is_state_stale(self, now_time) -> bool:
        age = self.last_state_age_sec(now_time)
        if age is None:
            return True
        if age < 0.0:
            return True
        return age > self.adapter_config.state_stale_threshold_sec

    def _rmf_update_handle(self):
        if self.update_handle is None:
            return None

        handle = self.update_handle
        more = getattr(handle, 'more', None)
        if callable(more):
            try:
                handle = more()
            except Exception as err:
                self.node.get_logger().warn(
                    f'[{self.name}] Failed to resolve RobotUpdateHandle via more(): {err}'
                )
                handle = self.update_handle
        return handle

    def _set_commissioned_state(self, commissioned: bool) -> bool:
        handle = self._rmf_update_handle()
        if handle is None:
            return False

        if commissioned:
            unstable = getattr(handle, 'unstable_recommission', None)
        else:
            unstable = getattr(handle, 'unstable_decommission', None)
        if callable(unstable):
            unstable()
            return True

        set_commission = getattr(handle, 'set_commission', None)
        get_commission = getattr(handle, 'commission', None)
        if callable(set_commission) and callable(get_commission):
            commission = get_commission()
            if not hasattr(commission, 'accept_dispatched_tasks'):
                return False
            commission.accept_dispatched_tasks = commissioned
            set_commission(commission)
            return True

        return False

    def _override_status(self, new_status: Optional[str]) -> bool:
        handle = self._rmf_update_handle()
        if handle is None:
            return False

        override_status = getattr(handle, 'override_status', None)
        if not callable(override_status):
            return False
        override_status(new_status)
        return True

    def _request_replan(self):
        handle = self._rmf_update_handle()
        if handle is None:
            return

        replan = getattr(handle, 'replan', None)
        if callable(replan):
            replan()

    def _set_unavailable(self, reason: str):
        if reason in self._decommission_reasons:
            return
        self._decommission_reasons.add(reason)
        if self.update_handle is None:
            return
        if not self._set_commissioned_state(False):
            self.node.get_logger().warn(
                f'[{self.name}] Could not decommission robot via RMF API'
            )
        self._override_status(self._unavailable_status_override())
        self.node.get_logger().warn(
            f'[{self.name}] decommissioned ({reason})'
        )

    def _clear_unavailable(self, reason: str):
        if reason not in self._decommission_reasons:
            return
        self._decommission_reasons.remove(reason)
        if self.update_handle is None:
            return
        if not self._decommission_reasons:
            if not self._set_commissioned_state(True):
                self.node.get_logger().warn(
                    f'[{self.name}] Could not recommission robot via RMF API'
                )
            self._override_status(None)
            self.node.get_logger().info(
                f'[{self.name}] recommissioned'
            )
        else:
            self._override_status(self._unavailable_status_override())

    def _unavailable_status_override(self) -> str:
        # Keep RMF status override values within schema enum values.
        if 'stale' in self._decommission_reasons:
            return 'offline'
        return 'error'

    def make_callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )

    def navigate(self, destination, execution):
        self._preempt_active('navigate')
        self.cmd_id += 1
        with self._cmd_lock:
            self.execution = execution
        dock = destination.dock if destination.dock is not None else None
        category = 'dock' if dock is not None else 'navigate'
        timeout_sec = self._compute_navigation_timeout(destination)
        self._start_command_tracking(category, timeout_sec)
        self._log_command_event('sent', self.cmd_id, category, {
            'map': destination.map,
            'dock': dock,
            'timeout_sec': timeout_sec,
        })

        self.attempt_cmd_until_success(
            cmd=self.api.navigate,
            args=(
                self.name,
                self.cmd_id,
                destination.position,
                destination.map,
                destination.speed_limit,
                dock,
            ),
            cmd_id=self.cmd_id,
            category=category,
        )

    def stop(self, activity):
        with self._cmd_lock:
            execution = self.execution
        if execution is not None:
            if execution.identifier.is_same(activity):
                self._log_command_event('stop', self.cmd_id, 'stop', {})
                execution.finished()
                self._finish_active_command(success=False, reason='stopped')
                with self._cmd_lock:
                    self.execution = None
                    self.active_command = None
        self.attempt_cmd_until_success(
            cmd=self.api.stop,
            args=(self.name,),
            cmd_id=self.cmd_id,
            category='stop',
        )

    def execute_action(self, category: str, description: dict, execution):
        if category not in self.adapter_config.allowed_actions:
            self.node.get_logger().error(
                f'[{self.name}] Rejected unsupported action category [{category}]'
            )
            execution.finished()
            return
        if not isinstance(description, dict):
            self.node.get_logger().error(
                f'[{self.name}] Invalid action payload for [{category}]'
            )
            execution.finished()
            self.metrics.record(category, 'rejected_invalid', 0.0)
            return
        required_keys = self.adapter_config.required_action_keys.get(category, [])
        missing = [key for key in required_keys if key not in description]
        if missing:
            self.node.get_logger().error(
                f'[{self.name}] Missing keys for [{category}]: {missing}'
            )
            execution.finished()
            self.metrics.record(category, 'rejected_invalid', 0.0)
            return
        if (
            self.adapter_config.enable_battery_gating
            and self.last_battery_soc is not None
            and self.last_battery_soc < self.adapter_config.low_battery_threshold
            and category not in self.adapter_config.critical_action_categories
        ):
            self.node.get_logger().warn(
                f'[{self.name}] Rejecting action [{category}] due to low battery'
            )
            execution.finished()
            self.metrics.record(category, 'rejected_low_battery', 0.0)
            return
        self._preempt_active(category)
        self.cmd_id += 1
        with self._cmd_lock:
            self.execution = execution
        timeout_sec = self._compute_action_timeout(category, description)
        self._start_command_tracking(category, timeout_sec)
        self._log_command_event('sent', self.cmd_id, category, {
            'timeout_sec': timeout_sec,
        })

        self.attempt_cmd_until_success(
            cmd=self.api.perform_action,
            args=(self.name, self.cmd_id, category, description),
            cmd_id=self.cmd_id,
            category=category,
        )

    def finish_action(self):
        with self._cmd_lock:
            execution = self.execution
        if execution is not None:
            execution.finished()
            self._finish_active_command(success=True, reason='manual_finish')
            with self._cmd_lock:
                self.execution = None
                self.active_command = None

    def attempt_cmd_until_success(self, cmd, args, cmd_id: int, category: str):
        if self.pending_cmd_id == cmd_id and self.pending_category == category:
            return
        self.cancel_cmd_attempt()
        self.pending_cmd_id = cmd_id
        self.pending_category = category

        def loop():
            while True:
                if cmd(*args):
                    self._log_command_event('accepted', cmd_id, category, {})
                    break
                self.node.get_logger().warn(
                    f'Failed to contact command endpoint for robot {self.name} '
                    f'(cmd_id={cmd_id}, category={category})'
                )
                if self.cancel_cmd_event.wait(self.adapter_config.retry_backoff_sec):
                    break
            self.pending_cmd_id = None
            self.pending_category = None

        self.issue_cmd_thread = threading.Thread(target=loop, args=())
        self.issue_cmd_thread.start()

    def cancel_cmd_attempt(self):
        if self.issue_cmd_thread is not None:
            self.cancel_cmd_event.set()
            if self.issue_cmd_thread.is_alive():
                self.issue_cmd_thread.join()
                self.issue_cmd_thread = None
        self.cancel_cmd_event.clear()

    def _start_command_tracking(self, category: str, timeout_sec: float):
        with self._cmd_lock:
            self.active_command = ActiveCommand(
                cmd_id=self.cmd_id,
                category=category,
                start_time=time.time(),
                timeout_sec=timeout_sec,
                execution=self.execution,
            )

    def _finish_active_command(self, success: bool, reason: str):
        with self._cmd_lock:
            active = self.active_command
        if active is None:
            return
        duration = time.time() - active.start_time
        outcome = 'success' if success else reason
        if reason in ['timeout', 'preempted', 'stopped']:
            outcome = reason
        self.metrics.record(active.category, outcome, duration)
        self._log_command_event('finished', active.cmd_id, active.category, {
            'success': success,
            'reason': reason,
            'duration_sec': duration,
        })
        if not success:
            self.fleet_handle.more().reassign_dispatched_tasks()

    def _check_timeout(self):
        with self._cmd_lock:
            active = self.active_command
            execution = self.execution
        if active is None:
            return
        elapsed = time.time() - active.start_time
        if elapsed <= active.timeout_sec:
            return
        self.node.get_logger().warn(
            f'[{self.name}] Command {active.cmd_id} timed out '
            f'after {elapsed:.1f}s'
        )
        self.api.stop(self.name)
        if execution is not None:
            execution.finished()
        self._finish_active_command(success=False, reason='timeout')
        with self._cmd_lock:
            self.execution = None
            self.active_command = None

    def _compute_navigation_timeout(self, destination) -> float:
        base = self.adapter_config.navigate_timeout_base_sec
        per_meter = self.adapter_config.navigate_timeout_per_meter_sec
        distance = 0.0
        if self.last_position is not None:
            dx = destination.position[0] - self.last_position[0]
            dy = destination.position[1] - self.last_position[1]
            distance = math.sqrt(dx * dx + dy * dy)
        timeout = base + distance * per_meter
        timeout = max(timeout, self.adapter_config.navigate_timeout_min_sec)
        timeout = min(timeout, self.adapter_config.navigate_timeout_max_sec)
        return timeout

    def _compute_action_timeout(self, category: str, description: dict) -> float:
        if category in self.adapter_config.action_timeouts:
            return float(self.adapter_config.action_timeouts[category])
        if isinstance(description, dict):
            if description.get('duration_ms') is not None:
                return float(description['duration_ms']) / 1000.0
            if description.get('duration_sec') is not None:
                return float(description['duration_sec'])
        return float(self.adapter_config.default_action_timeout_sec)

    def _preempt_active(self, new_category: str):
        with self._cmd_lock:
            execution = self.execution
            active = self.active_command
        if execution is None or active is None:
            return
        self.node.get_logger().warn(
            f'[{self.name}] Preempting cmd_id={active.cmd_id} '
            f'for new command category={new_category}'
        )
        self.api.stop(self.name)
        execution.finished()
        self._finish_active_command(success=False, reason='preempted')
        with self._cmd_lock:
            self.execution = None
            self.active_command = None

    def _log_command_event(self, event: str, cmd_id: int, category: str, extra: dict):
        payload = {
            'event': event,
            'robot': self.name,
            'cmd_id': cmd_id,
            'category': category,
        }
        payload.update(extra or {})
        self.node.get_logger().info(f'CMD {json.dumps(payload)}')


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935

def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(
            None, f, *args, **kwargs
        )

    return run_in_parallel


@parallel
def update_robot(robot: RobotAdapter):
    data = robot.api.get_data(robot.name)
    if data is None:
        return

    if data.map_name:
        robot.last_map_name = data.map_name

    if robot.update_handle is None:
        battery_soc = data.battery_soc
        if not data.battery_soc_valid:
            battery_soc = float(robot.adapter_config.battery_fallback_soc)
        map_name = data.map_name or robot.last_map_name or 'unknown'
        state = rmf_easy.RobotState(map_name, data.position, battery_soc)
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name, state, robot.configuration, robot.make_callbacks()
        )
        return

    robot.update(data)


def ros_connections(node, robots, fleet_handle):
    fleet_name = fleet_handle.more().fleet_name

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    closed_lanes_pub = node.create_publisher(
        ClosedLanes, 'closed_lanes', qos_profile=transient_qos
    )

    closed_lanes = set()

    def lane_request_cb(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            return

        fleet_handle.more().open_lanes(msg.open_lanes)
        fleet_handle.more().close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)

        for lane_idx in msg.open_lanes:
            if lane_idx in closed_lanes:
                closed_lanes.remove(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        closed_lanes_pub.publish(state_msg)

    def speed_limit_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        requests = []
        for limit in msg.speed_limits:
            request = rmf_adapter.fleet_update_handle.SpeedLimitRequest(
                limit.lane_index, limit.speed_limit)
            requests.append(request)
        fleet_handle.more().limit_lane_speeds(requests)
        fleet_handle.more().remove_speed_limits(msg.remove_limits)

    def mode_request_cb(msg):
        if (
            msg.fleet_name is None
            or msg.fleet_name != fleet_name
            or msg.robot_name is None
        ):
            return

        if msg.mode.mode == RobotMode.MODE_IDLE:
            robot = robots.get(msg.robot_name)
            if robot is None:
                return
            robot.finish_action()

    lane_request_sub = node.create_subscription(
        LaneRequest,
        'lane_closure_requests',
        lane_request_cb,
        qos_profile=qos_profile_system_default,
    )

    speed_limit_request_sub = node.create_subscription(
        SpeedLimitRequest,
        'speed_limit_requests',
        speed_limit_request_cb,
        qos_profile=qos_profile_system_default,
    )

    action_execution_notice_sub = node.create_subscription(
        ModeRequest,
        'action_execution_notice',
        mode_request_cb,
        qos_profile=qos_profile_system_default,
    )

    return [
        lane_request_sub,
        speed_limit_request_sub,
        action_execution_notice_sub,
    ]


if __name__ == '__main__':
    main(sys.argv)
