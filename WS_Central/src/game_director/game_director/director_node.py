from __future__ import annotations

import copy
import json
import math
import os
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from ieee_fleet_msgs.msg import RobotState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_task_msgs.msg import DispatchState, DispatchStates, TaskSummary
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .dispatch_client import DispatchClient
from .dispatch_tracker import DispatchTracker
from .models import (
    CapabilityHealth,
    MissionPhase,
    TaskLifecycle,
    TaskRuntime,
)
from .task_pool import load_task_pool, summarize_task_pool
from .world_model import WorldModel


_PLACEHOLDER = re.compile(r'\{\{\s*([a-zA-Z0-9_.]+)\s*\}\}')
_DEFAULT_BOOT_VALIDATION_REQUIREMENTS = [
    'dispatch_states_publisher',
    'task_summaries_publisher',
    'task_api_requests_subscriber',
    'expected_robots_seen',
    'expected_robots_online',
    'valid_robot_maps',
    'finite_robot_pose',
    'sane_battery_values',
]


def _parse_phase(value: object) -> Optional[MissionPhase]:
    text = str(value).strip().upper()
    if not text:
        return None
    if text in {'ANTENNA', 'DUCK', 'CRATER', 'DRONE', 'ENDGAME'}:
        text = MissionPhase.GAME_START.value
    try:
        return MissionPhase(text)
    except Exception:
        return None


class GameDirector(Node):
    def __init__(self):
        super().__init__('game_director')

        self.declare_parameter('task_pool_path', '')
        self.declare_parameter('boot_validation_requirements', _DEFAULT_BOOT_VALIDATION_REQUIREMENTS)
        self.declare_parameter('robot_state_topic', '/ieee_fleet/robot_state')
        self.declare_parameter('external_event_topic', '/game_director/events')
        self.declare_parameter('status_topic', '/game_director/status')
        self.declare_parameter('final_report_topic', '/game_director/final_report')
        self.declare_parameter('start_service_name', '/game_director/start_game')
        self.declare_parameter('match_duration_sec', 0.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self._spec = load_task_pool(self._resolve_task_pool_path())
        self._settings = self._spec.settings

        match_duration_override = float(self.get_parameter('match_duration_sec').value)
        if match_duration_override > 0.0:
            self._settings.match_duration_sec = match_duration_override

        self._world = WorldModel(self._spec.world)
        self._phase: MissionPhase = MissionPhase.BOOT
        self._world.phase = self._phase
        self._boot_validation_requirements = self._load_boot_validation_requirements()

        self._tasks: Dict[str, TaskRuntime] = {
            task.task_id: TaskRuntime(definition=task)
            for task in self._spec.tasks
            if task.enabled
        }

        self._game_to_rmf: Dict[str, str] = {}
        self._rmf_to_game: Dict[str, str] = {}
        self._locks: Dict[str, str] = {}

        self._dispatch_tracker = DispatchTracker()
        self._dispatch_client = DispatchClient(
            self,
            response_timeout_sec=self._settings.api_response_timeout_sec,
        )

        self._capability_health: Dict[str, CapabilityHealth] = defaultdict(CapabilityHealth)
        self._dynamic_priority_boost: Dict[str, Tuple[float, float]] = {}
        self._final_report_published = False
        self._last_final_report_phase = ''
        self._start_requested = False

        self._boot_started_unix_s = self._now_unix_s()
        self._safe_hold_reason = ''

        robot_state_topic = str(self.get_parameter('robot_state_topic').value)
        event_topic = str(self.get_parameter('external_event_topic').value)
        start_service_name = str(self.get_parameter('start_service_name').value)

        self._status_pub = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        final_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self._final_report_pub = self.create_publisher(
            String,
            str(self.get_parameter('final_report_topic').value),
            final_qos,
        )

        self.create_subscription(RobotState, robot_state_topic, self._on_robot_state, 100)
        self.create_subscription(DispatchStates, 'dispatch_states', self._on_dispatch_states, self._services_qos())
        self.create_subscription(TaskSummary, 'task_summaries', self._on_task_summary, 100)
        self.create_subscription(String, event_topic, self._on_external_event, 20)
        self.create_service(Trigger, start_service_name, self._on_start_game_service)

        self._timer = self.create_timer(self._settings.tick_period_sec, self._tick)

        total_tasks, by_type = summarize_task_pool(list(task.definition for task in self._tasks.values()))
        self.get_logger().info(
            f'Loaded task pool with {total_tasks} tasks ({by_type}); '
            f'boot_wait={self._settings.boot_wait_sec:.1f}s tick={self._settings.tick_period_sec:.2f}s '
            f'boot_checks={sorted(self._boot_validation_requirements)}'
        )

    def _load_boot_validation_requirements(self) -> Set[str]:
        raw_param = self.get_parameter_or('boot_validation_requirements')
        if raw_param.type_ == Parameter.Type.NOT_SET:
            self.get_logger().warn(
                "boot_validation_requirements is uninitialized; "
                "treating it as an empty list (all BOOT checks disabled)."
            )
            raw = []
        else:
            raw = raw_param.value
        if not isinstance(raw, (list, tuple)):
            self.get_logger().warn(
                'boot_validation_requirements must be a string list; using defaults'
            )
            return set(_DEFAULT_BOOT_VALIDATION_REQUIREMENTS)

        enabled: Set[str] = set()
        invalid: List[str] = []
        for value in raw:
            key = str(value).strip()
            if not key:
                continue
            if key not in _DEFAULT_BOOT_VALIDATION_REQUIREMENTS:
                invalid.append(key)
                continue
            enabled.add(key)

        if invalid:
            self.get_logger().warn(
                f'Ignoring unknown boot_validation_requirements entries: {sorted(set(invalid))}'
            )

        if raw and not enabled:
            self.get_logger().warn(
                'No valid boot_validation_requirements left after filtering; using defaults'
            )
            return set(_DEFAULT_BOOT_VALIDATION_REQUIREMENTS)

        return enabled

    def _resolve_task_pool_path(self) -> Path:
        param_path = str(self.get_parameter('task_pool_path').value).strip()
        candidates: List[Path] = []
        if param_path:
            expanded = os.path.expandvars(param_path)
            candidates.append(Path(expanded).expanduser())
        candidates.append(Path.cwd() / 'config' / 'task_pool.yaml')
        try:
            share_dir = Path(get_package_share_directory('game_director'))
            candidates.append(share_dir / 'config' / 'task_pool.yaml')
        except Exception:
            pass
        module_file = Path(__file__).resolve()
        for parent in module_file.parents:
            candidates.append(parent / 'config' / 'task_pool.yaml')

        checked: List[str] = []
        seen: Set[str] = set()
        for candidate in candidates:
            resolved = candidate.expanduser()
            key = str(resolved)
            if key in seen:
                continue
            seen.add(key)
            checked.append(key)
            if resolved.exists():
                return resolved

        raise FileNotFoundError(
            f'Unable to locate task_pool.yaml; checked: {checked}'
        )

    @staticmethod
    def _services_qos() -> QoSProfile:
        return QoSProfile(
            history=History.KEEP_LAST,
            depth=20,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )

    def _tick(self):
        now = self._now_unix_s()

        self._world.refresh_heartbeats(now, self._settings.stale_robot_timeout_sec)
        self._cleanup_expired_priority_boosts(now)

        if self._phase == MissionPhase.BOOT:
            self._run_boot_stage(now)
            self._publish_status(now)
            return

        if self._phase == MissionPhase.SAFE_HOLD:
            self._enforce_safe_hold(now)
            if self._boot_checks_passed(now):
                self._safe_hold_reason = ''
                self._set_phase(MissionPhase.READY, 'safe hold checks recovered')
            self._publish_status(now)
            return

        if self._phase == MissionPhase.READY:
            if not self._boot_checks_passed(now):
                self._start_requested = False
                self._safe_hold_reason = 'ready validation failed'
                self._set_phase(MissionPhase.SAFE_HOLD, self._safe_hold_reason)
                self._publish_status(now)
                return

            if self._start_requested:
                if self._world.match_start_unix_s <= 0.0:
                    self._world.set_match_start(now)
                self._start_requested = False
                self._set_phase(MissionPhase.GAME_START, 'start_game service called')
            self._publish_status(now)
            return

        self._consume_api_responses(now)
        self._sync_rmf_state(now)
        self._enforce_watchdogs(now)
        self._recompute_task_states(now)
        self._advance_phase(now)

        self._dispatch_ready_tasks(now)

        if self._phase == MissionPhase.COMPLETE:
            self._publish_final_report_if_needed(now)

        self._publish_status(now)

    def _run_boot_stage(self, now: float):
        if self._boot_checks_passed(now):
            self._safe_hold_reason = ''
            self._set_phase(MissionPhase.READY, 'boot validation passed')
            return

        elapsed = now - self._boot_started_unix_s
        if elapsed > self._settings.boot_wait_sec:
            self._safe_hold_reason = 'boot validation timeout'
            self._set_phase(MissionPhase.SAFE_HOLD, self._safe_hold_reason)

    def _boot_checks_passed(self, now: float) -> bool:
        del now
        checks = self._boot_validation_requirements

        if 'dispatch_states_publisher' in checks and self.count_publishers('dispatch_states') <= 0:
            return False

        if 'task_summaries_publisher' in checks and self.count_publishers('task_summaries') <= 0:
            return False

        if 'task_api_requests_subscriber' in checks and self.count_subscribers('task_api_requests') <= 0:
            return False

        if 'expected_robots_seen' in checks and not self._world.all_expected_robots_seen():
            return False

        if 'expected_robots_online' in checks and not self._world.all_expected_robots_online():
            return False

        if 'valid_robot_maps' in checks and not self._world.valid_robot_maps():
            return False

        if 'finite_robot_pose' in checks:
            for robot in self._world.robots.values():
                if not math.isfinite(robot.x) or not math.isfinite(robot.y) or not math.isfinite(robot.yaw):
                    return False

        if 'sane_battery_values' in checks and not self._world.sane_battery_values():
            return False

        return True

    def _enforce_safe_hold(self, now: float):
        for runtime in self._tasks.values():
            if runtime.state in {TaskLifecycle.RUNNING, TaskLifecycle.SUBMITTED, TaskLifecycle.CANCELING}:
                self._begin_cancel(runtime, reason='safe_hold', now=now, outcome='hold')

    def _advance_phase(self, now: float):
        del now

        if self._phase == MissionPhase.GAME_START and self._all_tasks_terminal():
            self._set_phase(MissionPhase.COMPLETE, 'all mission tasks terminal')

    def _set_phase(self, phase: MissionPhase, reason: str):
        if phase == self._phase:
            return
        self._phase = phase
        self._world.phase = phase
        self.get_logger().info(f'Phase -> {phase.value} ({reason})')

    def _all_tasks_terminal(self) -> bool:
        return all(t.state.is_terminal() for t in self._tasks.values())

    def _task_terminal(self, task_id: str) -> bool:
        runtime = self._tasks.get(task_id)
        if runtime is None:
            return True
        return runtime.state.is_terminal()

    def _recompute_task_states(self, now: float):
        for runtime in self._tasks.values():
            if runtime.state in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING, TaskLifecycle.CANCELING}:
                continue

            if self._success_conditions_met(runtime):
                if runtime.state != TaskLifecycle.DONE:
                    self._mark_done(runtime)
                continue

            if self._dismiss_conditions_met(runtime):
                if runtime.state != TaskLifecycle.SKIPPED:
                    runtime.state = TaskLifecycle.SKIPPED
                    self._release_task_locks(runtime)
                continue

            if runtime.state in {TaskLifecycle.DONE, TaskLifecycle.SKIPPED, TaskLifecycle.CANCELED}:
                continue

            if runtime.state == TaskLifecycle.FAILED and not runtime.can_retry_now(now):
                continue

            if not self._phase_allows_task(runtime):
                runtime.state = TaskLifecycle.PENDING
                continue

            if runtime.attempts >= runtime.definition.retry.max_attempts:
                runtime.state = TaskLifecycle.FAILED
                continue

            dependency_state = self._dependency_resolution_state(runtime)
            if dependency_state == 'block':
                runtime.state = TaskLifecycle.PENDING
                continue
            if dependency_state == 'skip':
                runtime.state = TaskLifecycle.SKIPPED
                runtime.status_note = 'dependency failed'
                self._release_task_locks(runtime)
                continue
            if dependency_state == 'fail':
                runtime.state = TaskLifecycle.FAILED
                runtime.last_error = 'dependency failed'
                runtime.status_note = 'dependency failed'
                self._release_task_locks(runtime)
                continue

            if self._preconditions_met(runtime):
                runtime.state = TaskLifecycle.READY
            else:
                runtime.state = TaskLifecycle.PENDING

    def _dependency_resolution_state(self, runtime: TaskRuntime) -> str:
        if not runtime.definition.depends_on:
            return 'ready'

        require_terminal = runtime.definition.dependency_policy == 'require_terminal'
        on_failure = runtime.definition.on_dependency_failure

        for task_id in runtime.definition.depends_on:
            other = self._tasks.get(task_id)
            if other is None:
                return 'fail'

            if other.state == TaskLifecycle.DONE:
                continue

            if require_terminal:
                if not other.state.is_terminal():
                    return 'block'
                if on_failure == 'run_anyway':
                    continue
                if other.state != TaskLifecycle.DONE:
                    return on_failure
                continue

            # require_done
            if other.state.is_terminal():
                if on_failure == 'run_anyway':
                    continue
                return on_failure if other.state != TaskLifecycle.DONE else 'ready'
            return 'block'

        return 'ready'

    def _phase_allows_task(self, runtime: TaskRuntime) -> bool:
        del runtime
        return self._phase == MissionPhase.GAME_START

    def _preconditions_met(self, runtime: TaskRuntime) -> bool:
        return self._conditions_match(runtime.definition.preconditions, runtime)

    def _dismiss_conditions_met(self, runtime: TaskRuntime) -> bool:
        if not runtime.definition.dismiss_conditions:
            return False
        return self._conditions_match(runtime.definition.dismiss_conditions, runtime)

    def _success_conditions_met(self, runtime: TaskRuntime) -> bool:
        if not runtime.definition.success_conditions:
            return False
        return self._conditions_match(runtime.definition.success_conditions, runtime)

    def _conditions_match(self, conditions: List[Dict[str, Any]], runtime: TaskRuntime) -> bool:
        for condition in conditions:
            if not self._condition_holds(condition, runtime):
                return False
        return True

    def _condition_holds(self, condition: Dict[str, Any], runtime: TaskRuntime) -> bool:
        kind = str(condition.get('kind', '')).strip()
        if kind == 'always':
            return True

        if kind == 'phase_is':
            target_phase = _parse_phase(condition.get('phase', ''))
            if target_phase is None:
                return False
            return self._phase == target_phase

        if kind == 'phase_in':
            values = [_parse_phase(v) for v in condition.get('phases', [])]
            values = [v for v in values if v is not None]
            return self._phase in values

        if kind == 'any_antenna_done':
            return self._world.any_antenna_done()

        if kind == 'all_antennas_terminal':
            return self._world.all_antennas_terminal()

        if kind == 'all_ducks_in_drop_zone':
            return self._world.all_ducks_in_drop_zone()

        if kind == 'antenna_state_in':
            antenna_id = str(condition.get('antenna_id', runtime.definition.target.get('antenna_id', '')))
            states = {str(v).upper() for v in condition.get('states', [])}
            antenna = self._world.antennas.get(antenna_id)
            return antenna is not None and antenna.value in states

        if kind == 'duck_state_in':
            duck_id = str(condition.get('duck_id', runtime.definition.target.get('duck_id', '')))
            states = {str(v).upper() for v in condition.get('states', [])}
            duck = self._world.ducks.get(duck_id)
            return duck is not None and duck.state.value in states

        if kind == 'task_done':
            task_id = str(condition.get('task_id', ''))
            other = self._tasks.get(task_id)
            return other is not None and other.state == TaskLifecycle.DONE

        if kind == 'task_terminal':
            task_id = str(condition.get('task_id', ''))
            other = self._tasks.get(task_id)
            return other is not None and other.state.is_terminal()

        if kind == 'crater_token_free':
            owner = self._locks.get('crater_token')
            return owner in {None, runtime.definition.task_id}

        if kind == 'lock_free':
            key_template = str(condition.get('lock', ''))
            key = self._render_string(key_template, self._template_context(runtime, '', '', self._now_unix_s()))
            owner = self._locks.get(key)
            return owner in {None, runtime.definition.task_id}

        if kind == 'robot_online':
            robot = str(condition.get('robot', ''))
            snapshot = self._world.robots.get(robot)
            return snapshot is not None and snapshot.online

        if kind == 'any_ground_robot_available':
            for robot_name in self._candidate_ground_robots(runtime):
                if self._robot_available_for_task(robot_name, runtime):
                    return True
            return False

        if kind == 'any_drone_available':
            for robot_name in self._candidate_drone_robots(runtime):
                if self._robot_available_for_task(robot_name, runtime):
                    return True
            return False

        if kind == 'match_time_before_sec':
            max_sec = float(condition.get('value', 0.0))
            return self._world.elapsed_match_time(self._now_unix_s()) <= max_sec

        if kind == 'match_time_after_sec':
            min_sec = float(condition.get('value', 0.0))
            return self._world.elapsed_match_time(self._now_unix_s()) >= min_sec

        return False

    def _dispatch_ready_tasks(self, now: float):
        candidates = [runtime for runtime in self._tasks.values() if runtime.state == TaskLifecycle.READY]
        if not candidates:
            return

        candidates.sort(key=lambda runtime: self._priority_score(runtime, now), reverse=True)

        submissions = 0
        max_submissions_per_tick = 4
        for runtime in candidates:
            if submissions >= max_submissions_per_tick:
                break

            if not runtime.can_retry_now(now):
                continue
            if runtime.definition.task_type == 'CRATER_LAP_PAIR' and not self._crater_pair_dispatchable():
                continue

            selected_robot, selected_fleet = self._select_robot(runtime)
            if runtime.definition.dispatch.mode == 'robot_targeted' and not selected_robot:
                continue

            lock_keys = self._render_lock_keys(runtime, selected_robot, selected_fleet, now)
            if not self._acquire_locks(runtime.definition.task_id, lock_keys):
                continue

            request = self._render_request(runtime, selected_robot, selected_fleet, now)
            try:
                request_id = self._dispatch_client.send_dispatch_request(
                    game_task_id=runtime.definition.task_id,
                    request=request,
                    mode=runtime.definition.dispatch.mode,
                    fleet=selected_fleet,
                    robot=selected_robot,
                    now_unix_s=now,
                )
            except Exception as exc:
                self._release_keys(lock_keys)
                runtime.last_error = f'dispatch submit failed: {exc}'
                runtime.state = TaskLifecycle.PENDING
                continue

            runtime.state = TaskLifecycle.SUBMITTED
            runtime.attempts += 1
            runtime.request_id = request_id
            runtime.submitted_at_unix_s = now
            runtime.assigned_robot = selected_robot
            runtime.assigned_fleet = selected_fleet
            runtime.held_locks = lock_keys
            runtime.status_note = 'waiting_api_response'
            runtime.cancel_reason = ''
            runtime.cancel_outcome = ''
            runtime.cancel_since_unix_s = 0.0
            runtime.cancel_requested_at_unix_s = 0.0
            if runtime.definition.task_type == 'CRATER_LAP_PAIR':
                self._world.crater.lap_in_progress = True

            submissions += 1
            self.get_logger().info(
                f'Submitted {runtime.definition.task_id} '
                f'(attempt={runtime.attempts}, mode={runtime.definition.dispatch.mode}, '
                f'robot={selected_robot or "<rmf>"}, fleet={selected_fleet or "<any>"})'
            )

    def _crater_pair_dispatchable(self) -> bool:
        crater_tasks = [runtime for runtime in self._tasks.values() if runtime.definition.task_type == 'CRATER_LAP_PAIR']
        if len(crater_tasks) <= 1:
            return True

        for runtime in crater_tasks:
            if runtime.state in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING}:
                continue
            if runtime.state != TaskLifecycle.READY:
                return False

            selected_robot, _ = self._select_robot(runtime)
            if runtime.definition.dispatch.mode == 'robot_targeted' and not selected_robot:
                return False

        return True

    def _priority_score(self, runtime: TaskRuntime, now: float) -> float:
        score = runtime.definition.priority.base

        remaining = max(0.0, self._settings.match_duration_sec - self._world.elapsed_match_time(now))
        score += runtime.definition.priority.deadline_boost_per_sec * (self._settings.match_duration_sec - remaining)

        dynamic = self._dynamic_priority_boost.get(runtime.definition.task_id)
        if dynamic and now <= dynamic[1]:
            score += dynamic[0]

        if runtime.definition.phase == self._phase:
            score += 10.0

        if runtime.definition.task_type == 'CLEAR_BLOCKING_DUCK':
            score += 30.0

        return score

    def _cleanup_expired_priority_boosts(self, now: float):
        to_remove = [task_id for task_id, (_, expiry) in self._dynamic_priority_boost.items() if now > expiry]
        for task_id in to_remove:
            self._dynamic_priority_boost.pop(task_id, None)

    def _select_robot(self, runtime: TaskRuntime) -> Tuple[str, str]:
        mode = runtime.definition.dispatch.mode
        if mode == 'robot_targeted':
            desired_robot = runtime.definition.dispatch.robot
            if desired_robot and self._robot_available_for_task(desired_robot, runtime):
                return desired_robot, runtime.definition.dispatch.fleet or self._infer_fleet(runtime, desired_robot)

            preferred = runtime.definition.preferred_robots or runtime.definition.allowed_robots
            for robot in preferred:
                if self._robot_available_for_task(robot, runtime):
                    return robot, runtime.definition.dispatch.fleet or self._infer_fleet(runtime, robot)
            return '', runtime.definition.dispatch.fleet or self._infer_fleet(runtime, '')

        # best-available
        candidates = self._candidate_robots_for_task(runtime)
        best_robot = ''
        best_score = -1.0
        for robot in candidates:
            if not self._robot_available_for_task(robot, runtime):
                continue
            health = self._world.robot_health_score(
                robot,
                self._settings.min_battery_soc,
                self._capability_health[robot].failures_by_type,
            )
            if health <= 0.0:
                continue

            score = health
            if robot in runtime.definition.preferred_robots:
                score += 0.4
            inflight_major, inflight_micro = self._inflight_counts_for_robot(robot)
            score -= 0.2 * inflight_major
            score -= 0.1 * inflight_micro

            if runtime.definition.task_type.startswith('DRONE_'):
                if robot in self._world.expected_drone_robots:
                    score += 0.6
                else:
                    score -= 0.6

            if score > best_score:
                best_score = score
                best_robot = robot

        fleet = runtime.definition.dispatch.fleet
        if not fleet and best_robot:
            fleet = self._infer_fleet(runtime, best_robot)
        return best_robot, fleet or ''

    def _candidate_robots_for_task(self, runtime: TaskRuntime) -> List[str]:
        if runtime.definition.allowed_robots:
            return list(runtime.definition.allowed_robots)

        if runtime.definition.task_type.startswith('DRONE_'):
            if self._world.expected_drone_robots:
                return list(self._world.expected_drone_robots)
            return [name for name in self._world.robots if name.startswith('drone')]

        if self._world.expected_ground_robots:
            return list(self._world.expected_ground_robots)
        return [name for name in self._world.robots if not name.startswith('drone')]

    def _candidate_ground_robots(self, runtime: TaskRuntime) -> List[str]:
        del runtime
        if self._world.expected_ground_robots:
            return list(self._world.expected_ground_robots)
        return [name for name in self._world.robots if not name.startswith('drone')]

    def _candidate_drone_robots(self, runtime: TaskRuntime) -> List[str]:
        del runtime
        if self._world.expected_drone_robots:
            return list(self._world.expected_drone_robots)
        return [name for name in self._world.robots if name.startswith('drone')]

    def _robot_available_for_task(self, robot_name: str, runtime: TaskRuntime) -> bool:
        if not robot_name:
            return False

        snapshot = self._world.robots.get(robot_name)
        if snapshot is None or not snapshot.online:
            return False

        if runtime.definition.excluded_robots and robot_name in runtime.definition.excluded_robots:
            return False

        if runtime.definition.allowed_robots and robot_name not in runtime.definition.allowed_robots:
            return False

        if runtime.definition.required_capabilities:
            capabilities = self._world.robot_capabilities.get(robot_name, set())
            for capability in runtime.definition.required_capabilities:
                if capability not in capabilities:
                    return False

        if snapshot.battery_valid and snapshot.battery_soc < self._settings.min_battery_soc:
            return False

        if runtime.definition.task_type in self._capability_health[robot_name].degraded_types:
            return False

        inflight_major, inflight_micro = self._inflight_counts_for_robot(robot_name)
        if runtime.definition.major and inflight_major >= self._settings.major_inflight_limit_per_robot:
            return False
        if not runtime.definition.major and inflight_micro >= self._settings.micro_inflight_limit_per_robot:
            return False

        if runtime.definition.task_type in {'COLLECT_DUCK', 'CLEAR_BLOCKING_DUCK'} and snapshot.carrying_duck_id:
            return False

        return True

    def _inflight_counts_for_robot(self, robot_name: str) -> Tuple[int, int]:
        major = 0
        micro = 0
        for runtime in self._tasks.values():
            if runtime.assigned_robot != robot_name:
                continue
            if runtime.state not in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING, TaskLifecycle.CANCELING}:
                continue
            if runtime.definition.major:
                major += 1
            else:
                micro += 1
        return major, micro

    def _infer_fleet(self, runtime: TaskRuntime, robot_name: str) -> str:
        if runtime.definition.dispatch.fleet:
            return runtime.definition.dispatch.fleet

        if runtime.definition.task_type.startswith('DRONE_'):
            return 'drone'

        if robot_name and robot_name in self._world.expected_drone_robots:
            return 'drone'

        return 'ground'

    def _render_lock_keys(self, runtime: TaskRuntime, robot: str, fleet: str, now: float) -> List[str]:
        keys: List[str] = []
        context = self._template_context(runtime, robot, fleet, now)
        for template in runtime.definition.lock_keys:
            keys.append(self._render_string(template, context))
        return keys

    def _acquire_locks(self, task_id: str, keys: List[str]) -> bool:
        for key in keys:
            owner = self._locks.get(key)
            if owner and owner != task_id:
                return False
        for key in keys:
            self._locks[key] = task_id
            if key == 'crater_token':
                self._world.crater.token_owner_task_id = task_id
        return True

    def _release_keys(self, keys: List[str]):
        for key in keys:
            self._locks.pop(key, None)
            if key == 'crater_token' and key not in self._locks:
                self._world.crater.token_owner_task_id = ''

    def _release_task_locks(self, runtime: TaskRuntime):
        self._release_keys(runtime.held_locks)
        runtime.held_locks.clear()

    def _template_context(self, runtime: TaskRuntime, robot: str, fleet: str, now: float) -> Dict[str, Any]:
        target = dict(runtime.definition.target)
        context: Dict[str, Any] = {
            'task_id': runtime.definition.task_id,
            'task_type': runtime.definition.task_type,
            'selected_robot': robot,
            'selected_fleet': fleet,
            'drop_zone': self._world.drop_zone,
            'now_unix_ms': int(now * 1000.0),
            'target': target,
        }
        context.update(target)
        return context

    def _render_request(self, runtime: TaskRuntime, robot: str, fleet: str, now: float) -> Dict[str, Any]:
        request = copy.deepcopy(runtime.definition.dispatch.request)
        request = self._render_template(request, self._template_context(runtime, robot, fleet, now))

        if 'category' not in request or 'description' not in request:
            raise ValueError(f'task {runtime.definition.task_id} dispatch.request requires category+description')

        start_ms = int(now * 1000.0)
        request.setdefault('unix_millis_request_time', start_ms)
        request.setdefault('unix_millis_earliest_start_time', start_ms)
        request.setdefault('requester', self._settings.requester)

        if runtime.definition.dispatch.mode == 'best_available' and fleet:
            request.setdefault('fleet_name', fleet)

        labels = list(request.get('labels', []))
        labels.extend([
            f'game_task:{runtime.definition.task_id}',
            f'game_type:{runtime.definition.task_type}',
            f'game_phase:{runtime.definition.phase.value}',
        ])
        request['labels'] = sorted(set(labels))

        return request

    def _render_template(self, value: Any, context: Dict[str, Any]) -> Any:
        if isinstance(value, dict):
            return {k: self._render_template(v, context) for k, v in value.items()}
        if isinstance(value, list):
            return [self._render_template(v, context) for v in value]
        if isinstance(value, str):
            return self._render_string(value, context)
        return value

    def _render_string(self, text: str, context: Dict[str, Any]) -> Any:
        matches = list(_PLACEHOLDER.finditer(text))
        if not matches:
            return text

        # If the whole string is one placeholder, preserve original type.
        if len(matches) == 1 and matches[0].span() == (0, len(text)):
            return self._resolve_context_path(matches[0].group(1), context)

        result = text
        for match in matches:
            key = match.group(1)
            value = self._resolve_context_path(key, context)
            result = result.replace(match.group(0), str(value))
        return result

    def _resolve_context_path(self, key: str, context: Dict[str, Any]) -> Any:
        current: Any = context
        for token in key.split('.'):
            if isinstance(current, dict) and token in current:
                current = current[token]
                continue
            return ''
        return current

    def _consume_api_responses(self, now: float):
        for runtime in self._tasks.values():
            if not runtime.request_id:
                continue

            response = self._dispatch_client.consume_response(runtime.request_id)
            if response is None:
                continue

            request_id = runtime.request_id
            runtime.request_id = ''

            if bool(response.get('success', False)):
                rmf_task_id = self._extract_rmf_task_id(response)
                if not rmf_task_id:
                    self._mark_failed(runtime, 'accepted response missing booking.id', now)
                    continue

                runtime.rmf_task_id = rmf_task_id
                self._game_to_rmf[runtime.definition.task_id] = rmf_task_id
                self._rmf_to_game[rmf_task_id] = runtime.definition.task_id
                runtime.dispatch_status = 'queued'
                runtime.status_note = 'accepted'
                continue

            reason = self._response_error_text(response)
            self._mark_failed(runtime, reason, now)

        for pending in self._dispatch_client.consume_pending_timeouts(now):
            runtime = self._tasks.get(pending.game_task_id)
            if runtime is None:
                continue
            if runtime.request_id != pending.request_id:
                continue
            runtime.request_id = ''
            self._mark_failed(runtime, 'api response timeout', now)

    def _extract_rmf_task_id(self, response: Dict[str, Any]) -> str:
        state = response.get('state', {})
        if isinstance(state, dict):
            booking = state.get('booking', {})
            if isinstance(booking, dict):
                task_id = booking.get('id', '')
                if task_id:
                    return str(task_id)
        return ''

    def _response_error_text(self, response: Dict[str, Any]) -> str:
        errors = response.get('errors', [])
        if isinstance(errors, list) and errors:
            first = errors[0]
            if isinstance(first, dict):
                category = first.get('category', '')
                detail = first.get('detail', '')
                return f'{category}: {detail}'.strip(': ')
            return str(first)
        return 'request rejected'

    def _sync_rmf_state(self, now: float):
        for rmf_task_id, game_task_id in list(self._rmf_to_game.items()):
            runtime = self._tasks.get(game_task_id)
            if runtime is None:
                continue

            dispatch = self._dispatch_tracker.dispatch_by_task_id.get(rmf_task_id)
            if dispatch:
                runtime.dispatch_status = dispatch.status_name
                if dispatch.robot_name:
                    runtime.assigned_robot = dispatch.robot_name
                if dispatch.fleet_name:
                    runtime.assigned_fleet = dispatch.fleet_name

                if dispatch.status in {DispatchState.STATUS_FAILED_TO_ASSIGN, DispatchState.STATUS_CANCELED_IN_FLIGHT}:
                    reason = '; '.join(dispatch.errors) if dispatch.errors else dispatch.status_name
                    if runtime.state == TaskLifecycle.CANCELING and runtime.cancel_outcome == 'hold':
                        self._mark_canceled_hold(runtime, now, reason)
                    else:
                        self._mark_failed(runtime, f'dispatch {reason}', now)
                    continue

            summary = self._dispatch_tracker.summary_by_task_id.get(rmf_task_id)
            if not summary:
                continue

            if summary.robot_name:
                runtime.assigned_robot = summary.robot_name
            if summary.fleet_name:
                runtime.assigned_fleet = summary.fleet_name

            if summary.state in {TaskSummary.STATE_PENDING, TaskSummary.STATE_QUEUED}:
                if runtime.state not in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING, TaskLifecycle.CANCELING}:
                    runtime.state = TaskLifecycle.SUBMITTED
                runtime.status_note = summary.status
                continue

            if summary.state == TaskSummary.STATE_ACTIVE:
                if runtime.state not in {TaskLifecycle.RUNNING, TaskLifecycle.CANCELING}:
                    runtime.state = TaskLifecycle.RUNNING
                    runtime.running_since_unix_s = summary.updated_at_unix_s
                runtime.status_note = summary.status
                continue

            if summary.state == TaskSummary.STATE_COMPLETED:
                self._mark_done(runtime)
                continue

            if summary.state in {TaskSummary.STATE_FAILED, TaskSummary.STATE_CANCELED}:
                reason = summary.status or summary.state_name
                if runtime.state == TaskLifecycle.CANCELING and runtime.cancel_outcome == 'hold':
                    self._mark_canceled_hold(runtime, now, reason)
                else:
                    cancel_reason = runtime.cancel_reason if runtime.state == TaskLifecycle.CANCELING else ''
                    self._mark_failed(runtime, cancel_reason or reason, now)
                continue

    def _enforce_watchdogs(self, now: float):
        for runtime in self._tasks.values():
            if runtime.state == TaskLifecycle.SUBMITTED and runtime.rmf_task_id:
                elapsed = now - runtime.submitted_at_unix_s
                if elapsed > runtime.definition.timeouts.queued_sec:
                    self._begin_cancel(runtime, reason='queued_timeout', now=now, outcome='retry')
                    continue

            if runtime.state == TaskLifecycle.RUNNING:
                elapsed = now - runtime.running_since_unix_s
                if elapsed > runtime.definition.timeouts.running_sec:
                    self._begin_cancel(runtime, reason='run_timeout', now=now, outcome='retry')
                    continue

            if runtime.state in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING} and runtime.assigned_robot:
                snapshot = self._world.robots.get(runtime.assigned_robot)
                if snapshot is None or not snapshot.online:
                    self._begin_cancel(runtime, reason='robot_offline', now=now, outcome='retry')
                    continue

            if runtime.state == TaskLifecycle.CANCELING:
                if (
                    runtime.cancel_requested_at_unix_s > 0.0
                    and now - runtime.cancel_requested_at_unix_s >= self._settings.cancel_retry_period_sec
                ):
                    self._request_cancel(runtime, reason=runtime.cancel_reason or 'canceling')
                    runtime.cancel_requested_at_unix_s = now

                if (
                    runtime.cancel_since_unix_s > 0.0
                    and now - runtime.cancel_since_unix_s >= self._settings.cancel_timeout_sec
                ):
                    if runtime.cancel_outcome == 'hold':
                        self._mark_canceled_hold(runtime, now, 'cancel timeout')
                    else:
                        self._mark_failed(runtime, 'cancel timeout', now, force_terminal=True)

    def _request_cancel(self, runtime: TaskRuntime, reason: str):
        if runtime.rmf_task_id:
            if runtime.dispatch_status in {'queued', 'selected'}:
                self._dispatch_client.send_cancel_service_request(runtime.rmf_task_id)
            self._dispatch_client.send_cancel_api_request(
                runtime.rmf_task_id,
                labels=[
                    f'game_task:{runtime.definition.task_id}',
                    f'reason:{reason}',
                ],
            )

    def _begin_cancel(self, runtime: TaskRuntime, reason: str, now: float, outcome: str):
        if runtime.state not in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING, TaskLifecycle.CANCELING}:
            return

        if runtime.state != TaskLifecycle.CANCELING:
            runtime.state = TaskLifecycle.CANCELING
            runtime.cancel_since_unix_s = now
            runtime.cancel_reason = reason
            runtime.cancel_outcome = outcome
            runtime.status_note = f'canceling:{reason}'
        elif outcome == 'hold':
            runtime.cancel_outcome = 'hold'
            if not runtime.cancel_reason:
                runtime.cancel_reason = reason

        should_send = (
            runtime.cancel_requested_at_unix_s <= 0.0
            or now - runtime.cancel_requested_at_unix_s >= self._settings.cancel_retry_period_sec
        )
        if should_send:
            self._request_cancel(runtime, reason=reason)
            runtime.cancel_requested_at_unix_s = now

    def _mark_canceled_hold(self, runtime: TaskRuntime, now: float, reason: str):
        runtime.status_note = f'canceled:{reason}'
        runtime.last_error = ''
        if runtime.attempts > 0:
            runtime.attempts -= 1
        runtime.blocked_until_unix_s = now
        if runtime.definition.task_type == 'CRATER_LAP_PAIR':
            self._world.crater.lap_in_progress = False
            self._world.crater.token_owner_task_id = ''
        runtime.state = TaskLifecycle.PENDING
        self._clear_runtime_tracking(runtime, release_locks=True)

    def _mark_done(self, runtime: TaskRuntime):
        runtime.state = TaskLifecycle.DONE
        runtime.status_note = 'completed'
        self._world.on_task_success(runtime.definition.task_type, runtime.definition.target, runtime.assigned_robot)
        self._record_success(runtime)
        self._clear_runtime_tracking(runtime, release_locks=True)

    def _mark_failed(
        self,
        runtime: TaskRuntime,
        reason: str,
        now: float,
        propagate: bool = True,
        force_terminal: bool = False,
    ):
        runtime.last_error = reason

        if propagate and runtime.definition.task_type == 'CRATER_LAP_PAIR':
            self._abort_crater_partner(runtime, reason, now)

        self._record_failure(runtime)
        attempts_exhausted = force_terminal or runtime.attempts >= runtime.definition.retry.max_attempts
        self._world.on_task_failure(
            runtime.definition.task_type,
            runtime.definition.target,
            attempts_exhausted=attempts_exhausted,
            reason=reason,
        )

        if runtime.definition.task_type == 'ACTIVATE_ANTENNA' and 'duck' in reason.lower() and 'block' in reason.lower():
            self._boost_duck_clearance(now)

        runtime.status_note = reason
        self._clear_runtime_tracking(runtime, release_locks=True)

        if attempts_exhausted:
            runtime.state = TaskLifecycle.FAILED
            return

        runtime.state = TaskLifecycle.PENDING
        runtime.mark_retry_backoff(now)

    def _abort_crater_partner(self, failed_runtime: TaskRuntime, reason: str, now: float):
        for runtime in self._tasks.values():
            if runtime.definition.task_type != 'CRATER_LAP_PAIR':
                continue
            if runtime.definition.task_id == failed_runtime.definition.task_id:
                continue
            if runtime.state not in {TaskLifecycle.SUBMITTED, TaskLifecycle.RUNNING, TaskLifecycle.CANCELING}:
                continue
            self._begin_cancel(
                runtime,
                reason='crater_partner_failed',
                now=now,
                outcome='retry',
            )

    def _clear_runtime_tracking(self, runtime: TaskRuntime, release_locks: bool):
        if runtime.rmf_task_id:
            self._rmf_to_game.pop(runtime.rmf_task_id, None)
        self._game_to_rmf.pop(runtime.definition.task_id, None)
        runtime.request_id = ''
        runtime.rmf_task_id = ''
        runtime.dispatch_status = ''
        runtime.running_since_unix_s = 0.0
        runtime.submitted_at_unix_s = 0.0
        runtime.cancel_reason = ''
        runtime.cancel_outcome = ''
        runtime.cancel_since_unix_s = 0.0
        runtime.cancel_requested_at_unix_s = 0.0
        if release_locks:
            self._release_task_locks(runtime)

    def _record_success(self, runtime: TaskRuntime):
        if runtime.assigned_robot:
            health = self._capability_health[runtime.assigned_robot]
            if runtime.definition.task_type in health.failures_by_type:
                failures = max(0, health.failures_by_type[runtime.definition.task_type] - 1)
                health.failures_by_type[runtime.definition.task_type] = failures
                if failures < self._settings.failure_degrade_threshold:
                    health.degraded_types.pop(runtime.definition.task_type, None)

    def _record_failure(self, runtime: TaskRuntime):
        if not runtime.assigned_robot:
            return

        health = self._capability_health[runtime.assigned_robot]
        failures = health.failures_by_type.get(runtime.definition.task_type, 0) + 1
        health.failures_by_type[runtime.definition.task_type] = failures
        if failures >= self._settings.failure_degrade_threshold:
            health.degraded_types[runtime.definition.task_type] = True
            self.get_logger().warn(
                f'Degraded capability: robot={runtime.assigned_robot} type={runtime.definition.task_type} '
                f'(failures={failures})'
            )

    def _boost_duck_clearance(self, now: float):
        for runtime in self._tasks.values():
            if runtime.definition.task_type in {'CLEAR_BLOCKING_DUCK', 'COLLECT_DUCK'}:
                boost = runtime.definition.priority.opportunistic_boost
                if boost <= 0.0:
                    boost = 50.0
                self._dynamic_priority_boost[runtime.definition.task_id] = (boost, now + 30.0)

    def _publish_status(self, now: float):
        counts: Dict[str, int] = defaultdict(int)
        for runtime in self._tasks.values():
            counts[runtime.state.value] += 1

        robot_status = {}
        for name, robot in self._world.robots.items():
            robot_status[name] = {
                'online': robot.online,
                'map': robot.map_name,
                'soc': robot.battery_soc if robot.battery_valid else None,
                'faults': list(robot.fault_flags),
            }

        payload = {
            'phase': self._phase.value,
            'elapsed_match_sec': round(self._world.elapsed_match_time(now), 2),
            'tasks': dict(counts),
            'locks': dict(self._locks),
            'robots': robot_status,
            'safe_hold_reason': self._safe_hold_reason,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _publish_final_report_if_needed(self, now: float):
        if self._last_final_report_phase == self._phase.value:
            return

        payload = {
            'timestamp_unix_s': now,
            'phase': self._phase.value,
            'antennas': {k: v.value for k, v in self._world.antennas.items()},
            'ducks': {k: v.state.value for k, v in self._world.ducks.items()},
            'crater': {
                'lap_in_progress': self._world.crater.lap_in_progress,
                'entry_clear': self._world.crater.entry_clear,
                'exit_clear': self._world.crater.exit_clear,
                'paired_robots': list(self._world.crater.paired_robots),
            },
            'drone': {k: v.value for k, v in self._world.drone_states.items()},
            'task_results': {
                runtime.definition.task_id: {
                    'state': runtime.state.value,
                    'attempts': runtime.attempts,
                    'last_error': runtime.last_error,
                    'rmf_task_id': runtime.rmf_task_id,
                }
                for runtime in self._tasks.values()
            },
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._final_report_pub.publish(msg)
        self._last_final_report_phase = self._phase.value

        if self._phase == MissionPhase.COMPLETE:
            self._final_report_published = True

        if not self._final_report_published:
            self.get_logger().info('Published final mission report')

    def _on_robot_state(self, msg: RobotState):
        self._world.update_robot_state(msg, self._now_unix_s())

    def _on_dispatch_states(self, msg: DispatchStates):
        self._dispatch_tracker.on_dispatch_states(msg, self._now_unix_s())

    def _on_task_summary(self, msg: TaskSummary):
        self._dispatch_tracker.on_task_summary(msg, self._now_unix_s())

    def _on_external_event(self, msg: String):
        event = self._world.apply_external_event(msg.data)
        if event == 'invalid_json':
            self.get_logger().warn('Ignored malformed /game_director/events payload')

    def _on_start_game_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._phase == MissionPhase.READY:
            self._start_requested = True
            response.success = True
            response.message = 'accepted: GAME_START requested'
            return response

        if self._phase == MissionPhase.GAME_START:
            response.success = True
            response.message = 'already in GAME_START'
            return response

        response.success = False
        response.message = f'cannot start from phase={self._phase.value}; wait for READY'
        return response

    def _now_unix_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9



def main(argv: List[str] = sys.argv):
    rclpy.init(args=argv)
    node = GameDirector()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
