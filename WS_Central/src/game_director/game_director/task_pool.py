from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml

from .models import (
    DispatchTemplate,
    MissionPhase,
    RetryPolicy,
    TaskDefinition,
    TaskPriority,
    TaskTimeouts,
)


@dataclass
class DirectorSettings:
    requester: str = 'game_director'
    tick_period_sec: float = 0.5
    boot_wait_sec: float = 10.0
    stale_robot_timeout_sec: float = 2.5
    min_battery_soc: float = 0.2
    major_inflight_limit_per_robot: int = 1
    micro_inflight_limit_per_robot: int = 1
    failure_degrade_threshold: int = 3
    match_duration_sec: float = 600.0
    queued_timeout_default_sec: float = 30.0
    running_timeout_default_sec: float = 120.0
    api_response_timeout_sec: float = 8.0
    cancel_retry_period_sec: float = 5.0
    cancel_timeout_sec: float = 45.0


@dataclass
class WorldSeed:
    antennas: Dict[str, str] = field(default_factory=dict)
    ducks: List[str] = field(default_factory=list)
    crater_pair: List[str] = field(default_factory=list)
    drop_zone: str = 'drop_zone'
    expected_ground_robots: List[str] = field(default_factory=list)
    expected_drone_robots: List[str] = field(default_factory=list)


@dataclass
class TaskPoolSpec:
    settings: DirectorSettings
    world: WorldSeed
    tasks: List[TaskDefinition]


def load_task_pool(path: Path) -> TaskPoolSpec:
    with path.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}

    settings = _parse_settings(raw.get('settings') or {})
    world = _parse_world(raw.get('world') or {})

    tasks: List[TaskDefinition] = []
    seen_ids = set()
    raw_tasks = raw.get('tasks')
    if raw_tasks is None:
        raw_tasks = []
    if not isinstance(raw_tasks, list):
        raise ValueError("task_pool field 'tasks' must be a list")

    for raw_task in raw_tasks:
        if not isinstance(raw_task, dict):
            raise ValueError("each entry in 'tasks' must be a mapping")
        task = _parse_task(raw_task, settings)
        if task.task_id in seen_ids:
            raise ValueError(f'duplicate task id in task pool: [{task.task_id}]')
        seen_ids.add(task.task_id)
        tasks.append(task)

    return TaskPoolSpec(settings=settings, world=world, tasks=tasks)


def _parse_settings(raw: Dict[str, Any]) -> DirectorSettings:
    return DirectorSettings(
        requester=str(raw.get('requester', 'game_director')),
        tick_period_sec=float(raw.get('tick_period_sec', 0.5)),
        boot_wait_sec=float(raw.get('boot_wait_sec', 10.0)),
        stale_robot_timeout_sec=float(raw.get('stale_robot_timeout_sec', 2.5)),
        min_battery_soc=float(raw.get('min_battery_soc', 0.2)),
        major_inflight_limit_per_robot=int(raw.get('major_inflight_limit_per_robot', 1)),
        micro_inflight_limit_per_robot=int(raw.get('micro_inflight_limit_per_robot', 1)),
        failure_degrade_threshold=int(raw.get('failure_degrade_threshold', 3)),
        match_duration_sec=float(raw.get('match_duration_sec', 600.0)),
        queued_timeout_default_sec=float(raw.get('queued_timeout_default_sec', 30.0)),
        running_timeout_default_sec=float(raw.get('running_timeout_default_sec', 120.0)),
        api_response_timeout_sec=float(raw.get('api_response_timeout_sec', 8.0)),
        cancel_retry_period_sec=float(raw.get('cancel_retry_period_sec', 5.0)),
        cancel_timeout_sec=float(raw.get('cancel_timeout_sec', 45.0)),
    )


def _parse_world(raw: Dict[str, Any]) -> WorldSeed:
    antennas = raw.get('antennas', {})
    if isinstance(antennas, list):
        antennas = {str(v): 'UNKNOWN' for v in antennas}

    return WorldSeed(
        antennas={str(k): str(v) for k, v in (antennas or {}).items()},
        ducks=[str(v) for v in raw.get('ducks', [])],
        crater_pair=[str(v) for v in raw.get('crater_pair', [])],
        drop_zone=str(raw.get('drop_zone', 'drop_zone')),
        expected_ground_robots=[str(v) for v in raw.get('expected_ground_robots', [])],
        expected_drone_robots=[str(v) for v in raw.get('expected_drone_robots', [])],
    )


def _parse_task(raw: Dict[str, Any], settings: DirectorSettings) -> TaskDefinition:
    task_id = str(raw['id'])
    task_type = str(raw['type'])
    raw_phase = str(raw.get('phase', MissionPhase.GAME_START.value)).upper()
    # Backward compatibility: legacy stage values now map into GAME_START.
    if raw_phase in {'ANTENNA', 'DUCK', 'CRATER', 'DRONE', 'ENDGAME'}:
        raw_phase = MissionPhase.GAME_START.value
    phase = MissionPhase(raw_phase)
    target = dict(raw.get('target', {}))

    dispatch_raw = raw.get('dispatch', {})
    request_template = dispatch_raw.get('request', {})

    priority_raw = raw.get('priority', {})
    retry_raw = raw.get('retry', {})
    timeout_raw = raw.get('timeouts', {})

    dispatch_mode = str(dispatch_raw.get('mode', 'best_available')).strip()
    if dispatch_mode not in {'best_available', 'robot_targeted'}:
        raise ValueError(f'task [{task_id}] has invalid dispatch mode [{dispatch_mode}]')

    dependency_policy = str(raw.get('dependency_policy', 'require_done')).strip()
    if dependency_policy not in {'require_done', 'require_terminal'}:
        raise ValueError(
            f'task [{task_id}] has invalid dependency_policy [{dependency_policy}]'
        )

    on_dependency_failure = str(raw.get('on_dependency_failure', 'skip')).strip()
    if on_dependency_failure not in {'skip', 'fail', 'run_anyway'}:
        raise ValueError(
            f'task [{task_id}] has invalid on_dependency_failure [{on_dependency_failure}]'
        )

    return TaskDefinition(
        task_id=task_id,
        task_type=task_type,
        phase=phase,
        enabled=bool(raw.get('enabled', True)),
        major=bool(raw.get('major', True)),
        target=target,
        preconditions=list(raw.get('preconditions', [])),
        dismiss_conditions=list(raw.get('dismiss_conditions', [])),
        success_conditions=list(raw.get('success_conditions', [])),
        lock_keys=[str(v) for v in raw.get('lock_keys', [])],
        priority=TaskPriority(
            base=float(priority_raw.get('base', 1.0)),
            deadline_boost_per_sec=float(priority_raw.get('deadline_boost_per_sec', 0.0)),
            opportunistic_boost=float(priority_raw.get('opportunistic_boost', 0.0)),
        ),
        retry=RetryPolicy(
            max_attempts=int(retry_raw.get('max_attempts', 2)),
            backoff_sec=float(retry_raw.get('backoff_sec', 2.0)),
        ),
        timeouts=TaskTimeouts(
            queued_sec=float(timeout_raw.get('queued_sec', settings.queued_timeout_default_sec)),
            running_sec=float(timeout_raw.get('running_sec', settings.running_timeout_default_sec)),
        ),
        dispatch=DispatchTemplate(
            mode=dispatch_mode,
            fleet=dispatch_raw.get('fleet'),
            robot=dispatch_raw.get('robot'),
            request=request_template,
        ),
        allowed_robots=[str(v) for v in raw.get('allowed_robots', [])],
        preferred_robots=[str(v) for v in raw.get('preferred_robots', [])],
        required_capabilities=[str(v) for v in raw.get('required_capabilities', [])],
        excluded_robots=[str(v) for v in raw.get('excluded_robots', [])],
        depends_on=[str(v) for v in raw.get('depends_on', [])],
        dependency_policy=dependency_policy,
        on_dependency_failure=on_dependency_failure,
        metadata=dict(raw.get('metadata', {})),
    )


def summarize_task_pool(tasks: List[TaskDefinition]) -> Tuple[int, Dict[str, int]]:
    by_type: Dict[str, int] = {}
    for task in tasks:
        by_type[task.task_type] = by_type.get(task.task_type, 0) + 1
    return len(tasks), by_type
