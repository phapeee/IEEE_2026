from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class MissionPhase(str, Enum):
    BOOT = 'BOOT'
    SAFE_HOLD = 'SAFE_HOLD'
    READY = 'READY'
    GAME_START = 'GAME_START'
    COMPLETE = 'COMPLETE'


class TaskLifecycle(str, Enum):
    PENDING = 'PENDING'
    READY = 'READY'
    SUBMITTED = 'SUBMITTED'
    RUNNING = 'RUNNING'
    CANCELING = 'CANCELING'
    DONE = 'DONE'
    FAILED = 'FAILED'
    SKIPPED = 'SKIPPED'
    CANCELED = 'CANCELED'

    def is_terminal(self) -> bool:
        return self in {
            TaskLifecycle.DONE,
            TaskLifecycle.FAILED,
            TaskLifecycle.SKIPPED,
            TaskLifecycle.CANCELED,
        }


class AntennaState(str, Enum):
    UNKNOWN = 'UNKNOWN'
    ACTIVATED = 'ACTIVATED'
    ATTEMPTED_FAILED = 'ATTEMPTED_FAILED'
    BLOCKED = 'BLOCKED'


class DuckState(str, Enum):
    UNKNOWN = 'UNKNOWN'
    DETECTED = 'DETECTED'
    CLAIMED = 'CLAIMED'
    CARRIED = 'CARRIED'
    IN_DROP_ZONE = 'IN_DROP_ZONE'
    LOST = 'LOST'


class DroneState(str, Enum):
    ON_GROUND = 'ON_GROUND'
    AIRBORNE = 'AIRBORNE'
    SCANNING = 'SCANNING'
    REPORTED = 'REPORTED'


@dataclass
class RetryPolicy:
    max_attempts: int = 2
    backoff_sec: float = 2.0


@dataclass
class TaskTimeouts:
    queued_sec: float = 30.0
    running_sec: float = 90.0


@dataclass
class TaskPriority:
    base: float = 1.0
    deadline_boost_per_sec: float = 0.0
    opportunistic_boost: float = 0.0


@dataclass
class DispatchTemplate:
    mode: str = 'best_available'  # best_available | robot_targeted
    fleet: Optional[str] = None
    robot: Optional[str] = None
    request: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TaskDefinition:
    task_id: str
    task_type: str
    phase: MissionPhase
    enabled: bool = True
    major: bool = True
    target: Dict[str, Any] = field(default_factory=dict)
    preconditions: List[Dict[str, Any]] = field(default_factory=list)
    dismiss_conditions: List[Dict[str, Any]] = field(default_factory=list)
    success_conditions: List[Dict[str, Any]] = field(default_factory=list)
    lock_keys: List[str] = field(default_factory=list)
    priority: TaskPriority = field(default_factory=TaskPriority)
    retry: RetryPolicy = field(default_factory=RetryPolicy)
    timeouts: TaskTimeouts = field(default_factory=TaskTimeouts)
    dispatch: DispatchTemplate = field(default_factory=DispatchTemplate)
    allowed_robots: List[str] = field(default_factory=list)
    preferred_robots: List[str] = field(default_factory=list)
    required_capabilities: List[str] = field(default_factory=list)
    excluded_robots: List[str] = field(default_factory=list)
    depends_on: List[str] = field(default_factory=list)
    dependency_policy: str = 'require_done'  # require_done | require_terminal
    on_dependency_failure: str = 'skip'  # skip | fail | run_anyway
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TaskRuntime:
    definition: TaskDefinition
    state: TaskLifecycle = TaskLifecycle.PENDING
    attempts: int = 0
    last_error: str = ''
    blocked_until_unix_s: float = 0.0
    submitted_at_unix_s: float = 0.0
    running_since_unix_s: float = 0.0
    request_id: str = ''
    rmf_task_id: str = ''
    dispatch_status: str = ''
    assigned_robot: str = ''
    assigned_fleet: str = ''
    held_locks: List[str] = field(default_factory=list)
    status_note: str = ''
    cancel_reason: str = ''
    cancel_outcome: str = ''  # retry | hold
    cancel_since_unix_s: float = 0.0
    cancel_requested_at_unix_s: float = 0.0

    def can_retry_now(self, now_unix_s: float) -> bool:
        if self.attempts >= self.definition.retry.max_attempts:
            return False
        return now_unix_s >= self.blocked_until_unix_s

    def mark_retry_backoff(self, now_unix_s: float):
        self.blocked_until_unix_s = now_unix_s + self.definition.retry.backoff_sec


@dataclass
class RobotSnapshot:
    robot_name: str
    map_name: str = ''
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    battery_soc: float = 1.0
    battery_valid: bool = False
    requires_replan: bool = False
    last_completed_request: int = 0
    last_update_unix_s: float = 0.0
    online: bool = False
    fault_flags: List[str] = field(default_factory=list)
    carrying_duck_id: str = ''


@dataclass
class DuckSnapshot:
    duck_id: str
    state: DuckState = DuckState.UNKNOWN
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    claimed_by: str = ''
    carried_by: str = ''


@dataclass
class CraterSnapshot:
    token_owner_task_id: str = ''
    lap_in_progress: bool = False
    entry_clear: bool = True
    exit_clear: bool = True
    paired_robots: List[str] = field(default_factory=list)


@dataclass
class DispatchMapping:
    rmf_task_id: str
    game_task_id: str
    status: str = ''
    assigned_robot: str = ''
    assigned_fleet: str = ''
    updated_at_unix_s: float = 0.0
    errors: List[str] = field(default_factory=list)


@dataclass
class CapabilityHealth:
    failures_by_type: Dict[str, int] = field(default_factory=dict)
    degraded_types: Dict[str, bool] = field(default_factory=dict)
