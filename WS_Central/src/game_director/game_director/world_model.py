from __future__ import annotations

import json
import math
from typing import Dict, List, Optional

from ieee_fleet_msgs.msg import RobotState

from .models import (
    AntennaState,
    CraterSnapshot,
    DroneState,
    DuckSnapshot,
    DuckState,
    MissionPhase,
    RobotSnapshot,
)
from .task_pool import WorldSeed


class WorldModel:
    def __init__(self, seed: WorldSeed):
        self.match_start_unix_s: float = 0.0
        self.phase: MissionPhase = MissionPhase.BOOT

        self.robots: Dict[str, RobotSnapshot] = {}
        self.antennas: Dict[str, AntennaState] = {
            antenna_id: _to_antenna_state(state)
            for antenna_id, state in seed.antennas.items()
        }
        self.ducks: Dict[str, DuckSnapshot] = {
            duck_id: DuckSnapshot(duck_id=duck_id)
            for duck_id in seed.ducks
        }
        self.crater = CraterSnapshot()
        self.drone_states: Dict[str, DroneState] = {}
        self.robot_capabilities: Dict[str, set[str]] = {}

        self.expected_ground_robots = list(seed.expected_ground_robots)
        self.expected_drone_robots = list(seed.expected_drone_robots)
        self.drop_zone = seed.drop_zone

    def set_match_start(self, now_unix_s: float):
        self.match_start_unix_s = now_unix_s

    def elapsed_match_time(self, now_unix_s: float) -> float:
        if self.match_start_unix_s <= 0.0:
            return 0.0
        return max(0.0, now_unix_s - self.match_start_unix_s)

    def update_robot_state(self, msg: RobotState, now_unix_s: float):
        battery_soc = float(msg.battery_soc)
        battery_valid = math.isfinite(battery_soc) and 0.0 <= battery_soc <= 1.0

        robot = self.robots.get(msg.robot_name)
        if robot is None:
            robot = RobotSnapshot(robot_name=msg.robot_name)
            self.robots[msg.robot_name] = robot

        robot.map_name = msg.map_name
        robot.x = float(msg.x)
        robot.y = float(msg.y)
        robot.yaw = float(msg.yaw)
        robot.battery_soc = battery_soc
        robot.battery_valid = battery_valid
        robot.requires_replan = bool(msg.requires_replan)
        robot.last_completed_request = int(msg.last_completed_request)
        robot.last_update_unix_s = now_unix_s
        robot.online = True

    def refresh_heartbeats(self, now_unix_s: float, stale_threshold_sec: float):
        for robot in self.robots.values():
            age = now_unix_s - robot.last_update_unix_s
            robot.online = robot.last_update_unix_s > 0.0 and age <= stale_threshold_sec

    def all_expected_robots_seen(self) -> bool:
        expected = set(self.expected_ground_robots + self.expected_drone_robots)
        if not expected:
            return len(self.robots) > 0
        return expected.issubset(set(self.robots.keys()))

    def valid_robot_maps(self) -> bool:
        for robot in self.robots.values():
            if not robot.map_name:
                return False
        return True

    def all_expected_robots_online(self) -> bool:
        expected = self.expected_ground_robots + self.expected_drone_robots
        if not expected:
            return any(robot.online for robot in self.robots.values())

        for robot_name in expected:
            robot = self.robots.get(robot_name)
            if robot is None or not robot.online:
                return False
        return True

    def sane_battery_values(self) -> bool:
        for robot in self.robots.values():
            if not math.isfinite(robot.battery_soc):
                return False
            if robot.battery_soc < 0.0 or robot.battery_soc > 1.0:
                return False
        return True

    def antenna_remaining(self) -> List[str]:
        remaining = []
        for antenna_id, state in self.antennas.items():
            if state not in {AntennaState.ACTIVATED, AntennaState.ATTEMPTED_FAILED}:
                remaining.append(antenna_id)
        return remaining

    def unclaimed_ducks(self) -> List[str]:
        result = []
        for duck_id, duck in self.ducks.items():
            if duck.state in {DuckState.DETECTED, DuckState.UNKNOWN, DuckState.LOST} and not duck.claimed_by:
                result.append(duck_id)
        return result

    def robot_health_score(
        self,
        robot_name: str,
        min_soc: float,
        failure_counts: Optional[Dict[str, int]] = None,
    ) -> float:
        robot = self.robots.get(robot_name)
        if robot is None or not robot.online:
            return 0.0

        score = 1.0
        if robot.battery_valid:
            if robot.battery_soc < min_soc:
                return 0.0
            score *= 0.7 + 0.3 * max(0.0, min(1.0, robot.battery_soc))
        if robot.requires_replan:
            score *= 0.7
        if robot.fault_flags:
            score *= 0.5
        if failure_counts:
            score *= 1.0 / (1.0 + 0.2 * sum(failure_counts.values()))
        return max(0.0, score)

    def all_antennas_terminal(self) -> bool:
        return len(self.antenna_remaining()) == 0

    def all_ducks_in_drop_zone(self) -> bool:
        return all(d.state == DuckState.IN_DROP_ZONE for d in self.ducks.values())

    def crater_idle(self) -> bool:
        return not self.crater.lap_in_progress

    def all_drones_terminal(self) -> bool:
        if not self.drone_states:
            return False
        return all(s in {DroneState.ON_GROUND, DroneState.REPORTED} for s in self.drone_states.values())

    def any_antenna_done(self) -> bool:
        for state in self.antennas.values():
            if state in {AntennaState.ACTIVATED, AntennaState.ATTEMPTED_FAILED}:
                return True
        return False

    def mark_antenna_state(self, antenna_id: str, state: AntennaState):
        self.antennas[antenna_id] = state

    def ensure_duck(self, duck_id: str) -> DuckSnapshot:
        duck = self.ducks.get(duck_id)
        if duck is None:
            duck = DuckSnapshot(duck_id=duck_id)
            self.ducks[duck_id] = duck
        return duck

    def _clear_duck_from_robots(self, duck_id: str, keep_robot: str = ''):
        for robot_name, robot in self.robots.items():
            if keep_robot and robot_name == keep_robot:
                continue
            if robot.carrying_duck_id == duck_id:
                robot.carrying_duck_id = ''

    def on_task_success(self, task_type: str, target: Dict[str, str], assigned_robot: str = ''):
        if task_type in {'ACTIVATE_ANTENNA', 'CLEAR_BLOCKING_DUCK'}:
            antenna_id = target.get('antenna_id')
            if antenna_id:
                self.mark_antenna_state(antenna_id, AntennaState.ACTIVATED)

        if task_type in {'COLLECT_DUCK', 'CLEAR_BLOCKING_DUCK'}:
            duck_id = target.get('duck_id')
            if duck_id:
                duck = self.ensure_duck(duck_id)
                duck.state = DuckState.IN_DROP_ZONE
                duck.claimed_by = ''
                duck.carried_by = ''
                self._clear_duck_from_robots(duck_id)

        if task_type == 'CRATER_LAP_PAIR':
            self.crater.lap_in_progress = False
            self.crater.token_owner_task_id = ''

        if task_type == 'DRONE_SCAN_LEDS':
            for drone_name in self.drone_states:
                self.drone_states[drone_name] = DroneState.SCANNING

        if task_type == 'DRONE_SEND_IR':
            for drone_name in self.drone_states:
                self.drone_states[drone_name] = DroneState.REPORTED

        if task_type == 'DRONE_LAND':
            for drone_name in self.drone_states:
                self.drone_states[drone_name] = DroneState.ON_GROUND

        if assigned_robot:
            robot = self.robots.get(assigned_robot)
            if robot and task_type == 'COLLECT_DUCK':
                robot.carrying_duck_id = ''

    def on_task_failure(
        self,
        task_type: str,
        target: Dict[str, str],
        attempts_exhausted: bool,
        reason: str = '',
    ):
        if task_type == 'ACTIVATE_ANTENNA' and attempts_exhausted:
            antenna_id = target.get('antenna_id')
            if antenna_id:
                self.mark_antenna_state(antenna_id, AntennaState.ATTEMPTED_FAILED)

        if task_type == 'COLLECT_DUCK':
            duck_id = target.get('duck_id')
            if duck_id:
                duck = self.ensure_duck(duck_id)
                if duck.state != DuckState.IN_DROP_ZONE:
                    duck.state = DuckState.LOST
                    duck.claimed_by = ''
                    duck.carried_by = ''
                    self._clear_duck_from_robots(duck_id)

        if task_type == 'CRATER_LAP_PAIR':
            self.crater.lap_in_progress = False
            self.crater.token_owner_task_id = ''

    def apply_external_event(self, data: str) -> str:
        try:
            payload = json.loads(data)
        except Exception:
            return 'invalid_json'

        event = payload.get('event', '')
        if event == 'antenna_state':
            antenna_id = str(payload.get('antenna_id', ''))
            if antenna_id:
                self.mark_antenna_state(antenna_id, _to_antenna_state(payload.get('state', 'UNKNOWN')))
            return event

        if event == 'duck_detected':
            duck_id = str(payload.get('duck_id', ''))
            if duck_id:
                duck = self.ensure_duck(duck_id)
                duck.state = DuckState.DETECTED
                duck.x = _to_float(payload.get('x', duck.x), duck.x)
                duck.y = _to_float(payload.get('y', duck.y), duck.y)
                duck.confidence = _to_float(payload.get('confidence', duck.confidence), duck.confidence)
            return event

        if event == 'duck_claimed':
            duck_id = str(payload.get('duck_id', ''))
            robot_name = str(payload.get('robot_name', ''))
            if duck_id:
                duck = self.ensure_duck(duck_id)
                duck.state = DuckState.CLAIMED
                duck.claimed_by = robot_name
                duck.carried_by = ''
            return event

        if event == 'duck_carried':
            duck_id = str(payload.get('duck_id', ''))
            robot_name = str(payload.get('robot_name', ''))
            if duck_id:
                duck = self.ensure_duck(duck_id)
                duck.state = DuckState.CARRIED
                duck.claimed_by = robot_name
                duck.carried_by = robot_name
                self._clear_duck_from_robots(duck_id, keep_robot=robot_name)
                robot = self.robots.get(robot_name)
                if robot is not None:
                    robot.carrying_duck_id = duck_id
            return event

        if event == 'duck_in_drop_zone':
            duck_id = str(payload.get('duck_id', ''))
            if duck_id:
                duck = self.ensure_duck(duck_id)
                duck.state = DuckState.IN_DROP_ZONE
                duck.claimed_by = ''
                duck.carried_by = ''
                self._clear_duck_from_robots(duck_id)
            return event

        if event == 'duck_lost':
            duck_id = str(payload.get('duck_id', ''))
            if duck_id:
                duck = self.ensure_duck(duck_id)
                duck.state = DuckState.LOST
                duck.claimed_by = ''
                duck.carried_by = ''
                self._clear_duck_from_robots(duck_id)
            return event

        if event == 'crater_state':
            self.crater.entry_clear = bool(payload.get('entry_clear', self.crater.entry_clear))
            self.crater.exit_clear = bool(payload.get('exit_clear', self.crater.exit_clear))
            self.crater.lap_in_progress = bool(payload.get('lap_in_progress', self.crater.lap_in_progress))
            paired = payload.get('paired_robots', self.crater.paired_robots)
            if isinstance(paired, list):
                self.crater.paired_robots = [str(v) for v in paired]
            return event

        if event == 'drone_state':
            drone_name = str(payload.get('drone_name', 'drone1'))
            state = _to_drone_state(payload.get('state', 'ON_GROUND'))
            self.drone_states[drone_name] = state
            return event

        if event == 'robot_fault':
            robot_name = str(payload.get('robot_name', ''))
            fault = str(payload.get('fault', '')).strip()
            set_fault = bool(payload.get('set', True))
            robot = self.robots.get(robot_name)
            if robot is not None and fault:
                if set_fault and fault not in robot.fault_flags:
                    robot.fault_flags.append(fault)
                if not set_fault and fault in robot.fault_flags:
                    robot.fault_flags.remove(fault)
            return event

        if event == 'robot_capability':
            robot_name = str(payload.get('robot_name', ''))
            capability = str(payload.get('capability', '')).strip()
            set_capability = bool(payload.get('set', True))
            if robot_name and capability:
                caps = self.robot_capabilities.setdefault(robot_name, set())
                if set_capability:
                    caps.add(capability)
                else:
                    caps.discard(capability)
            return event

        return 'ignored'



def _to_antenna_state(value: object) -> AntennaState:
    text = str(value).upper()
    try:
        return AntennaState[text]
    except Exception:
        return AntennaState.UNKNOWN



def _to_drone_state(value: object) -> DroneState:
    text = str(value).upper()
    try:
        return DroneState[text]
    except Exception:
        return DroneState.ON_GROUND


def _to_float(value: object, default: float) -> float:
    try:
        parsed = float(value)
        if math.isfinite(parsed):
            return parsed
    except Exception:
        pass
    return default
