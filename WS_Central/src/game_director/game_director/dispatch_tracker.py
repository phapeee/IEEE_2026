from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from rmf_task_msgs.msg import DispatchState, DispatchStates, TaskSummary


@dataclass
class DispatchSnapshot:
    task_id: str
    status: int
    status_name: str
    fleet_name: str
    robot_name: str
    errors: List[str]
    updated_at_unix_s: float


@dataclass
class SummarySnapshot:
    task_id: str
    state: int
    state_name: str
    fleet_name: str
    robot_name: str
    status: str
    updated_at_unix_s: float


class DispatchTracker:
    def __init__(self):
        self.dispatch_by_task_id: Dict[str, DispatchSnapshot] = {}
        self.summary_by_task_id: Dict[str, SummarySnapshot] = {}

    def on_dispatch_states(self, msg: DispatchStates, now_unix_s: float):
        for state in list(msg.active) + list(msg.finished):
            self._consume_dispatch_state(state, now_unix_s)

    def on_task_summary(self, msg: TaskSummary, now_unix_s: float):
        snapshot = SummarySnapshot(
            task_id=msg.task_id,
            state=int(msg.state),
            state_name=summary_state_to_str(msg.state),
            fleet_name=msg.fleet_name,
            robot_name=msg.robot_name,
            status=msg.status,
            updated_at_unix_s=now_unix_s,
        )
        self.summary_by_task_id[msg.task_id] = snapshot

    def _consume_dispatch_state(self, state: DispatchState, now_unix_s: float):
        errors = [str(e) for e in state.errors]
        fleet_name = ''
        robot_name = ''
        if state.assignment.is_assigned:
            fleet_name = state.assignment.fleet_name
            robot_name = state.assignment.expected_robot_name

        snapshot = DispatchSnapshot(
            task_id=state.task_id,
            status=int(state.status),
            status_name=dispatch_state_to_str(state.status),
            fleet_name=fleet_name,
            robot_name=robot_name,
            errors=errors,
            updated_at_unix_s=now_unix_s,
        )
        self.dispatch_by_task_id[state.task_id] = snapshot


def dispatch_state_to_str(value: int) -> str:
    mapping = {
        DispatchState.STATUS_UNINITIALIZED: 'uninitialized',
        DispatchState.STATUS_QUEUED: 'queued',
        DispatchState.STATUS_SELECTED: 'selected',
        DispatchState.STATUS_DISPATCHED: 'dispatched',
        DispatchState.STATUS_FAILED_TO_ASSIGN: 'failed_to_assign',
        DispatchState.STATUS_CANCELED_IN_FLIGHT: 'canceled_in_flight',
    }
    return mapping.get(int(value), f'unknown_{value}')


def summary_state_to_str(value: int) -> str:
    mapping = {
        TaskSummary.STATE_QUEUED: 'queued',
        TaskSummary.STATE_ACTIVE: 'active',
        TaskSummary.STATE_COMPLETED: 'completed',
        TaskSummary.STATE_FAILED: 'failed',
        TaskSummary.STATE_CANCELED: 'canceled',
        TaskSummary.STATE_PENDING: 'pending',
    }
    return mapping.get(int(value), f'unknown_{value}')

