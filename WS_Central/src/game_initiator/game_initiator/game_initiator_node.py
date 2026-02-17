from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


@dataclass
class InputState:
    name: str
    gpio_line: int
    active_low: bool
    pullup: bool
    trigger_on: str
    stable_state: Optional[bool] = None
    pending_state: Optional[bool] = None
    pending_since: float = 0.0


class GameInitiatorNode(Node):
    def __init__(self) -> None:
        super().__init__('game_initiator')

        default_chip = 'gpiochip4' if Path('/dev/gpiochip4').exists() else 'gpiochip0'

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self._backend = str(self.declare_parameter('backend', 'gpioget').value)
        self._gpio_chip = str(self.declare_parameter('gpio_chip', default_chip).value)
        self._default_gpio_line = int(self.declare_parameter('gpio_line', 4).value)
        self._default_active_low = bool(self.declare_parameter('active_low', True).value)
        self._default_pullup = bool(self.declare_parameter('use_internal_pullup', False).value)
        self._default_trigger_on = str(self.declare_parameter('trigger_on', 'press').value)

        self._polling_frequency_hz = float(
            self.declare_parameter('polling_frequency_hz', 50.0).value
        )
        debounce_ms = float(self.declare_parameter('debounce_duration_ms', 30.0).value)
        self._debounce_duration_sec = max(debounce_ms / 1000.0, 0.0)
        self._gpioget_timeout_sec = max(
            float(self.declare_parameter('gpioget_timeout_sec', 0.2).value),
            0.01,
        )

        self._start_service_name = str(
            self.declare_parameter('start_service_name', '/game_director/start_game').value
        )
        self._service_wait_timeout_sec = max(
            float(self.declare_parameter('service_wait_timeout_sec', 1.0).value),
            0.0,
        )
        self._service_call_cooldown_sec = max(
            float(self.declare_parameter('service_call_cooldown_sec', 1.0).value),
            0.0,
        )
        self._call_once_after_success = bool(
            self.declare_parameter('call_once_after_success', True).value
        )
        self._shutdown_on_success = bool(
            self.declare_parameter('shutdown_on_success', False).value
        )

        self._prime_pullups = bool(self.declare_parameter('prime_pullups', True).value)
        self._prime_pullups_script = str(
            self.declare_parameter(
                'prime_pullups_script',
                'scripts/prime_gpio_pullups.sh',
            ).value
        )

        raw_names = self.declare_parameter('inputs', Parameter.Type.STRING_ARRAY).value
        input_names = [str(name) for name in raw_names] if isinstance(raw_names, list) else []
        self._inputs = self._configure_inputs(input_names)
        if not self._inputs:
            raise RuntimeError('No GPIO inputs configured.')

        self._last_read_error_time: Dict[str, float] = {}
        self._request_in_flight = False
        self._success_latched = False
        self._last_service_call_time = 0.0

        self._start_client = self.create_client(Trigger, self._start_service_name)
        self._prime_pullup_lines_if_requested()

        period = 1.0 / self._polling_frequency_hz if self._polling_frequency_hz > 0.0 else 0.02
        self._timer = self.create_timer(period, self._poll_inputs)

        self.get_logger().info(
            'game_initiator started: '
            f'backend={self._backend} chip={self._gpio_chip} '
            f'inputs={len(self._inputs)} poll={period:.3f}s '
            f'service={self._start_service_name} '
            f"shutdown_on_success={'true' if self._shutdown_on_success else 'false'}"
        )
        for state in self._inputs.values():
            self.get_logger().info(
                f"Input '{state.name}': line={state.gpio_line} "
                f"active_low={'true' if state.active_low else 'false'} "
                f"pullup={'true' if state.pullup else 'false'} "
                f"trigger_on={state.trigger_on}"
            )

    def _configure_inputs(self, input_names: list[str]) -> Dict[str, InputState]:
        states: Dict[str, InputState] = {}
        if not input_names:
            trigger_on = self._normalize_trigger_on(self._default_trigger_on, 'default')
            states['default'] = InputState(
                name='default',
                gpio_line=self._default_gpio_line,
                active_low=self._default_active_low,
                pullup=self._default_pullup,
                trigger_on=trigger_on,
            )
            return states

        for name in input_names:
            prefix = f'inputs.{name}'
            gpio_line = int(
                self.declare_parameter(f'{prefix}.gpio_line', self._default_gpio_line).value
            )
            active_low = bool(
                self.declare_parameter(f'{prefix}.active_low', self._default_active_low).value
            )
            pullup = bool(
                self.declare_parameter(f'{prefix}.pullup', self._default_pullup).value
            )
            trigger_on = self._normalize_trigger_on(
                str(self.declare_parameter(f'{prefix}.trigger_on', self._default_trigger_on).value),
                name,
            )
            states[name] = InputState(
                name=name,
                gpio_line=gpio_line,
                active_low=active_low,
                pullup=pullup,
                trigger_on=trigger_on,
            )
        return states

    def _normalize_trigger_on(self, trigger_on: str, input_name: str) -> str:
        normalized = str(trigger_on).strip().lower()
        mapping = {
            'press': 'press',
            'pressed': 'press',
            'active': 'press',
            'rise': 'press',
            'rising': 'press',
            'release': 'release',
            'released': 'release',
            'inactive': 'release',
            'fall': 'release',
            'falling': 'release',
            'both': 'both',
            'any': 'both',
        }
        if normalized in mapping:
            return mapping[normalized]

        self.get_logger().warn(
            f"Input '{input_name}' has unsupported trigger_on='{trigger_on}'; "
            "defaulting to 'press'"
        )
        return 'press'

    def _poll_inputs(self) -> None:
        for state in self._inputs.values():
            try:
                active = self._read_gpio_active(state)
            except Exception as exc:
                self._log_read_error_throttled(state.name, state.gpio_line, exc)
                continue
            self._apply_debounce_and_process(state, active)

    def _read_gpio_active(self, state: InputState) -> bool:
        backend = self._backend.strip().lower()
        if backend not in {'gpioget', 'auto'}:
            raise RuntimeError(f"Unsupported backend '{self._backend}'")

        cmd = ['gpioget']
        if state.pullup:
            cmd.append('--bias=pull-up')
        cmd.extend([self._gpio_chip, str(state.gpio_line)])

        completed = subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True,
            timeout=self._gpioget_timeout_sec,
        )
        raw_level = self._parse_gpioget_stdout(completed.stdout)
        if raw_level is None:
            raise RuntimeError(f"Unexpected gpioget output: '{completed.stdout.strip()}'")
        return (not raw_level) if state.active_low else raw_level

    @staticmethod
    def _parse_gpioget_stdout(stdout: str) -> Optional[bool]:
        tokens = stdout.strip().split()
        if not tokens:
            return None

        value = tokens[0].strip().lower()
        if value in {'1', 'true', 'high', 'active'}:
            return True
        if value in {'0', 'false', 'low', 'inactive'}:
            return False
        return None

    def _apply_debounce_and_process(self, state: InputState, active_now: bool) -> None:
        now = time.monotonic()

        if state.pending_state is None or state.pending_state != active_now:
            state.pending_state = active_now
            state.pending_since = now
            return

        if now - state.pending_since < self._debounce_duration_sec:
            return

        if state.stable_state is None:
            state.stable_state = active_now
            return

        if state.stable_state == active_now:
            return

        previous = state.stable_state
        state.stable_state = active_now
        self.get_logger().info(
            f"Input '{state.name}' transitioned "
            f"{'active' if previous else 'inactive'} -> "
            f"{'active' if active_now else 'inactive'}"
        )

        if self._is_trigger_event(state.trigger_on, previous, active_now):
            self._request_start_game(state.name, previous, active_now)

    @staticmethod
    def _is_trigger_event(trigger_on: str, previous: bool, current: bool) -> bool:
        if trigger_on == 'both':
            return previous != current
        if trigger_on == 'release':
            return previous and not current
        return (not previous) and current

    def _request_start_game(self, source: str, previous: bool, current: bool) -> None:
        if self._call_once_after_success and self._success_latched:
            return

        if self._request_in_flight:
            return

        now = time.monotonic()
        if now - self._last_service_call_time < self._service_call_cooldown_sec:
            return

        if not self._start_client.service_is_ready():
            ready = self._start_client.wait_for_service(timeout_sec=self._service_wait_timeout_sec)
            if not ready:
                self.get_logger().warn(
                    f"Input '{source}' triggered "
                    f"({'active' if previous else 'inactive'} -> "
                    f"{'active' if current else 'inactive'}) but service "
                    f'{self._start_service_name} is unavailable'
                )
                return

        self._request_in_flight = True
        self._last_service_call_time = now

        request = Trigger.Request()
        future = self._start_client.call_async(request)
        future.add_done_callback(lambda fut, source_name=source: self._on_start_response(source_name, fut))

        self.get_logger().info(
            f"Input '{source}' triggered; calling {self._start_service_name}"
        )

    def _on_start_response(self, source: str, future) -> None:
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"start_game call from input '{source}' failed: {exc}"
            )
            return

        if response is None:
            self.get_logger().error(
                f"start_game call from input '{source}' returned no response"
            )
            return

        if response.success:
            self.get_logger().info(
                f"start_game accepted from input '{source}': {response.message}"
            )
            if self._call_once_after_success:
                self._success_latched = True
            if self._shutdown_on_success:
                self.get_logger().info(
                    'shutdown_on_success=true; shutting down game_initiator node.'
                )
                if self._timer is not None:
                    self._timer.cancel()
                rclpy.shutdown()
        else:
            self.get_logger().warn(
                f"start_game rejected from input '{source}': {response.message}"
            )

    def _prime_pullup_lines_if_requested(self) -> None:
        if not self._prime_pullups:
            return

        pullup_lines = sorted({state.gpio_line for state in self._inputs.values() if state.pullup})
        if not pullup_lines:
            return

        script_path = self._resolve_existing_path(self._prime_pullups_script)
        if script_path is None:
            self.get_logger().warn(
                f"prime_pullups enabled but script '{self._prime_pullups_script}' "
                "not found; using inline gpioget priming"
            )
            self._prime_with_gpioget(pullup_lines)
            return

        cmd = ['bash', str(script_path), self._gpio_chip, *[str(line) for line in pullup_lines]]
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True, timeout=5.0)
            self.get_logger().info(
                f'Primed GPIO pull-ups via {script_path} for chip={self._gpio_chip} '
                f"lines={','.join(str(line) for line in pullup_lines)}"
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to run pull-up priming script ({exc}). '
                'Falling back to inline priming.'
            )
            self._prime_with_gpioget(pullup_lines)

    def _prime_with_gpioget(self, pullup_lines: list[int]) -> None:
        primed_lines: list[int] = []
        for line in pullup_lines:
            try:
                subprocess.run(
                    ['gpioget', '--bias=pull-up', self._gpio_chip, str(line)],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=max(self._gpioget_timeout_sec, 0.5),
                )
                primed_lines.append(line)
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to prime pull-up for line {line} on '
                    f'{self._gpio_chip}: {exc}'
                )
        if primed_lines:
            self.get_logger().info(
                f'Primed GPIO pull-ups with gpioget for chip={self._gpio_chip} '
                f"lines={','.join(str(line) for line in primed_lines)}"
            )

    def _resolve_existing_path(self, path_str: str) -> Optional[Path]:
        if not path_str:
            return None

        raw = Path(path_str).expanduser()
        candidates = [raw]
        if not raw.is_absolute():
            candidates.append(Path.cwd() / raw)

        for parent in Path(__file__).resolve().parents:
            candidates.append(parent / raw)
            candidates.append(parent / 'scripts' / 'prime_gpio_pullups.sh')

        for candidate in candidates:
            try:
                resolved = candidate.resolve()
            except Exception:
                resolved = candidate
            if resolved.is_file():
                return resolved

        return None

    def _log_read_error_throttled(self, name: str, line: int, exc: Exception) -> None:
        now = time.monotonic()
        last = self._last_read_error_time.get(name, 0.0)
        if now - last < 2.0:
            return
        self._last_read_error_time[name] = now
        self.get_logger().error(
            f"Failed reading input '{name}' (line {line} on {self._gpio_chip}): {exc}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GameInitiatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
