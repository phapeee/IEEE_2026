# gpio_button_event

`gpio_button_event` is a lightweight ROS 2 (rclcpp) package that monitors one or more GPIO lines via `libgpiod` and emits SMACC2 events whenever the hardware button or limit switch toggles. It also republishes the instantaneous state on configurable `std_msgs/Bool` topics so other nodes can inspect the level of each input.

## Features

- Uses `libgpiod` to access `/dev/gpiochip*` devices (the legacy `/sys/class/gpio` interface is no longer supported).
- Built-in debounce filter and optional active-low inversion.
- Publishes `smacc2_msgs/SmaccEvent` so SMACC2 state machines can react to button presses/releases.
- Provides a companion `std_msgs/Bool` topic for debugging and general consumption.
- Supports multiple named switches, each with its own GPIO line, SMACC event types, and optional state mirror topic.

## Dependencies

Install the SMACC2 interfaces and GPIO helpers before building the workspace. `libgpiod` is mandatory:

```bash
sudo apt install ros-humble-smacc2-msgs libgpiod-dev
```

## Build

From the workspace root:

```bash
colcon build --packages-select gpio_button_event --merge-install
source install/setup.bash
```

## Running the node

You can launch the default node, which observes GPIO line 4 on `gpiochip0`, with:

```bash
ros2 launch gpio_button_event gpio_button_event.launch.py
```

The launch file configures the node to publish on:

- `smacc2/button_event` (`smacc2_msgs/SmaccEvent`)
- `smacc2/button_state` (`std_msgs/Bool`)

The default SMACC2 payloads are:

| Transition | `event_type` | `event_object_tag` | `label` example |
|------------|--------------|-------------------|-----------------|
| Button pressed  | `BUTTON_PRESSED`  | `GPIO4Button` | `GPIO4Button:pressed` |
| Button released | `BUTTON_RELEASED` | `GPIO4Button` | `GPIO4Button:released` |

You can remap or override any of the parameters defined in `launch/gpio_button_event.launch.py`.

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `backend` | `string` | `gpiod` | Only `gpiod` is supported (the value is kept for backwards compatibility). |
| `gpio_chip` | `string` | `gpiochip4` if present, else `gpiochip0` | GPIOD chip name when `gpiod` backend is active. Ignored for sysfs. |
| `gpio_line` | `int` | `4` | GPIO line number to monitor when only one switch is configured. |
| `active_low` | `bool` | `false` | Treats logic low as pressed. |
| `use_internal_pullup` | `bool` | `false` | Requests the kernel to enable the internal pull-up (overridable per switch). |
| `polling_frequency_hz` | `double` | `50.0` | Sampling rate for the input. |
| `debounce_duration_ms` | `double` | `30.0` | Minimum stable time before a new state is accepted. |
| `smacc_event_topic` | `string` | `smacc2/button_event` | Topic for SMACC2 events. |
| `button_state_topic` | `string` | `smacc2/button_state` | Default state topic (shared by the only switch or used as the base for the first entry in `switches`). |
| `event_object_tag` | `string` | `GPIO4Button` | Populated in `SmaccEvent.event_object_tag` when only one switch is active. |
| `event_source` | `string` | `gpio_button_event_node` | Identifies the node/component in SMACC2 logs. |
| `pressed_event_type` | `string` | `BUTTON_PRESSED` | `SmaccEvent.event_type` sent on press in single-switch mode. |
| `released_event_type` | `string` | `BUTTON_RELEASED` | `SmaccEvent.event_type` sent on release in single-switch mode. |
| `switches` | `string[]` | `[]` | List of named switches (e.g. `["front_switch"]`). Each name enables `switches.<name>.*` parameters. |

### Configuring multiple switches

When `switches` is empty the node behaves exactly like the original single-button implementation. Populating the list enables per-switch parameters under `switches.<name>.*`:

- `gpio_line` (required): GPIO line number on `gpio_chip`.
- `active_low`: Overrides the top-level `active_low` for this switch.
- `pullup`: Requests the kernel pull-up bias for the line so you can wire the switch to ground.
- `event_object_tag`, `pressed_event_type`, `released_event_type`: Customize the SMACC payload for that line.
- `state_topic`: Optional `std_msgs/Bool` mirror for that switch. Leave blank to disable. By default the first switch reuses `button_state_topic` and subsequent entries append `/<name>`.

Example configuration for four limit switches wired with internal pull-ups:

```yaml
gpio_button_event:
  ros__parameters:
    backend: gpiod
    gpio_chip: "gpiochip0"
    button_state_topic: "smacc2/button_state"
    smacc_event_topic: "smacc2/button_event"
    switches: ["front_switch", "back_swtich", "left_switch", "right_switch"]
    switches.front_switch.gpio_line: 18
    switches.front_switch.active_low: true
    switches.front_switch.pullup: true
    switches.front_switch.event_object_tag: "front_switch"
    switches.front_switch.pressed_event_type: "front_switch"
    switches.front_switch.released_event_type: "front_switch_released"
    switches.front_switch.state_topic: "smacc2/button_state"
    switches.back_swtich.gpio_line: 24
    switches.back_swtich.active_low: true
    switches.back_swtich.pullup: true
    switches.back_swtich.event_object_tag: "back_swtich"
    switches.back_swtich.pressed_event_type: "back_swtich"
    switches.back_swtich.released_event_type: "back_swtich_released"
    switches.back_swtich.state_topic: "smacc2/back_swtich_state"
    # left_switch and right_switch defined similarly...
```

Each switch publishes its own `smacc2_msgs/SmaccEvent` with the provided event names, allowing your SMACC2 state machine to react to front/back/left/right limit switch transitions independently.

## Connecting to SMACC2

Point your SMACC2 state machine's event generator to the `smacc2/button_event` topic (or whichever topic you remap). The `event_type`, `event_object_tag`, and `label` fields allow you to distinguish button transitions from other event sources.

If you need a simple boolean indicator inside other nodes, subscribe to `smacc2/button_state`.

## Troubleshooting

- **Permission errors:** Accessing GPIO via sysfs or `libgpiod` often requires the `gpio` group or running the node with elevated privileges.
- **No state changes detected:** Confirm each switch is wired to the configured GPIO line and that `active_low` matches your pull-up/pull-down configuration.
- **Use a different pin:** Override `gpio_line` (and `gpio_chip` when necessary) through parameters or the launch file.
