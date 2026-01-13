# ROS 2 Workspace

This workspace contains the `diff_drive_controller` package and the `robot_main` launcher package.

## Launching the system

After building the workspace and sourcing `install/setup.bash`, run:

```bash
ros2 launch robot_main robot_main.launch.py
```

The launch file now reads every node/component definition from `config/robot_main.yaml`. Each entry under the top-level `components` block declares whether the component is enabled plus any node-specific arguments. Leave the `robot_main_config` argument unset to use this default file or point it to an alternative YAML when you want different combinations of components.

Set `enabled` to `false` to keep a component out of the launch description. Optional fields let you override names, controller-manager namespaces, parameter overrides, or remappings on a per-component basis. The optional top-level `namespace` entry defines the global namespace applied to every launched node; relative topic names are resolved inside this namespace so you can avoid repeating prefixes like `/bot_0` throughout the component definitions.

### IMU calibration helper

The workspace also ships an `imu_calibration` package that flattens IMU orientation during startup. Add the component to `config/robot_main.yaml` and point it at your desired topics (see `config/imu_calibration.yaml` for a template). When enabled, the node subscribes to the configured IMU topic, averages the first `calibration_samples` readings (or stops after `calibration_timeout_sec`), and then publishes calibrated orientations on `output_topic`.

To run it manually:

```bash
ros2 run imu_calibration imu_calibration_node \
  --ros-args -p input_topic:=/bot_0/imu_data -p output_topic:=/bot_0/imu/data_calibrated
```

Use the calibrated topic wherever you previously consumed the raw IMU message (e.g., in `robot_localization`).

## Sending velocity commands

Launch a keyboard teleop node (for example `teleop_twist_keyboard`) in another terminal to publish `geometry_msgs/Twist` messages to `/cmd_vel`. The `cmd_vel_relay` section in `config/robot_main.yaml` sets the relay node name, enable flag, and the `ros__parameters` block that defines the input/output topics (default `/cmd_vel` → `/mecanum_controller/reference`) plus the `frame_id` stamped on the outgoing `geometry_msgs/TwistStamped` messages:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The remaining controller manager nodes, spawners, and the `robot_state_publisher` are now launched automatically through `robot_main.launch.py`, so you can configure and enable them entirely via `robot_main.yaml` instead of running manual `ros2 run` commands.

### Offline data plotting

For quick visualization of the exported CSV data, run the standalone helper script (no ROS node needed):

```bash
python3 scripts/data_plot.py --csv config/uwb_config_samples.csv --headers x y
```

Each requested header produces a histogram with an overlaid normal distribution fit plus basic stats (count, mean, std, variance) embedded on the chart, making it easy to inspect how well your samples cluster before finalizing the covariance.

## OpenCR protocol test (USB link)

The OpenCR sketch exposes a small USB serial protocol (PING/ECHO/STATUS). Install the Python dependencies (pulls in `pyserial` for the tests) and then run the hardware-in-the-loop pytest to verify that the Pi and OpenCR can talk correctly:

```bash
pip install -e src/robot_main
python3 -m pytest -s -rs src/robot_main/test/test_opencr_protocol.py
```

Make sure the board is running the protocol (wheel-ID test disabled) and is connected over USB. Override the defaults with `OPENCR_SERIAL_PORT` (default `/dev/ttyACM0`) and `OPENCR_SERIAL_BAUD` (default `115200`) if needed.

## Nav2 navigation stack

`config/nav2_params.yaml` contains the tuned controller, planner, smoother, behavior server, BT navigator, waypoint follower, and velocity smoother settings. The default `robot_main.yaml` enables the `nav2_stack` component so those nodes are launched directly from `ros2 launch robot_main robot_main.launch.py`. The accompanying `nav2_lifecycle_manager_navigation` entry points at the same parameter file to drive lifecycle transitions for the Nav2 servers (autostarting them just like `nav2_bringup`). Update `nav2_params.yaml` and the `nodes` list or `node_overrides` under `nav2_stack` in `config/robot_main.yaml` if you want to add/remove plugins or remap topics.

The standalone `nav2_map_server` component continues to source `config/nav2_map_server.yaml`, and the lightweight `nav2_lifecycle_manager` component drives just that lifecycle node (so you still get a map even when the rest of Nav2 is disabled). Toggle any of these components in `config/robot_main.yaml` depending on whether you're running full navigation, localization-only, or bringup-without-autonomy scenarios.

## Button-triggered waypoint missions (SMACC2)

`src/smacc_button_nav` packages a SMACC2 state machine that ties the GPIO button to Nav2 waypoint runs. The flow is:

1. Start in `Idle`, waiting for a rising edge on `smacc2/button_state`.
2. On press, feed the next waypoint to Nav2 (`navigate_to_pose` action) and wait for success.
3. Pause at the waypoint for `wait_duration_sec` seconds, then continue with the remaining waypoints.
4. After the last waypoint the machine returns to `Idle` and waits for the next mission.
5. At any time, another button press jumps straight to a `Reset` state that cancels the active goal and rewinds the waypoint index (you can later extend `StReset` with your own reset behavior).

The component definition in `config/robot_main.yaml` wires everything together:

```yaml
  waypoint_state_machine:
    enabled: true
    package: smacc_button_nav
    executable: button_waypoint_sm
    ros__parameters:
      button_state_topic: "smacc2/button_state"
      navigate_action_name: "navigate_to_pose"
      waypoint_frame_id: "map"
      wait_duration_sec: 5.0
      waypoint_angles_in_degrees: true
      waypoints: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
```

Supply your own waypoint list as `[x, y, yaw]` triplets (yaw in radians, expressed in `waypoint_frame_id`). Build and launch once the `gpio_button_event`, Nav2 stack, and SMACC2 node are enabled:

```bash
colcon build --packages-select gpio_button_event smacc_button_nav robot_main --merge-install
source install/setup.bash
ros2 launch robot_main robot_main.launch.py
```

The SMACC2 node publishes its transitions through the normal SMACC2 introspection topics, so you can visualize the graph in SMACC Viewer while testing. Customize `StReset` later to add the reset reactions you need—the hooks are already in place. If you'd rather specify yaw angles in degrees, set `waypoint_angles_in_degrees` to `true` (as above) and the SMACC2 node will convert each value to radians before building the quaternion.

## Limit switch calibration state machines

The `limit_switch_calibration` package adds a flexible ROS 2 node that pushes the robot against walls with the four new limit switches, performs the requested motions, and hard-sets the pose in `robot_localization` when the scripted sequence reaches completion. Runtime topics/services (including `front_switch_topic`, `back_switch_topic`, `left_switch_topic`, `right_switch_topic`, and `cmd_vel_topic`) plus the path to the sequence file are configured in `config/limit_switch_calibration.yaml`, while the actual calibration routines live in `config/limit_switch_sequences.yaml`. This keeps the ROS 2 parameter file simple (so it can be loaded through `ros2 run ... --params-file`) while still letting you describe rich state machines in YAML.

Start a routine by publishing the desired machine ID on the `limit_switch_calibration/start` topic:

```bash
ros2 topic pub --once limit_switch_calibration/start std_msgs/String '{data: "front_wall_touch"}'
```

The node sequences three kinds of steps:

- **Conditions** wait for any/all switch states (`front_pressed`, `back_released`, etc.) or a pure timer. Provide `timeout_sec` to bound how long the wait lasts and optional `duration_sec` for the timer condition. Combine multiple conditions with `any_of`/`all_of` lists.
- **Movements** stream a constant velocity for a fixed duration. Specify `velocity_x` for forward/back motion, `velocity_y` for strafing, and `duration_sec` to hold the twist command. The node automatically publishes a zero twist when the timer expires.
- **Actions** currently support `set_pose` (call `robot_localization/SetPose` with the configured pose) and `terminate` (emit a SMACC2 event to report success/failure and stop the machine). The `pose` dictionary accepts `frame_id`, `x`, `y`, `yaw`/`yaw_deg`, and `covariance_diagonal` entries so you can feed the exact alignment you expect after touching the walls.

Multiple machines can coexist in `config/limit_switch_sequences.yaml`, each under its own ID. The example file ships two sequences: `front_wall_touch` drives forward until the front switch fires, sets the pose to `(0, 0, 0)`, and emits `CALIBRATION_SUCCESS`. `box_corner_square` demonstrates a richer script that strafes left, waits for both front/back switches, pauses with the timer condition, and then localizes with a 90° yaw. Adjust or add new machines to match your calibration steps; the node simply executes the ordered list of conditions, movements, and actions you supply. Point `state_machine_file` at an alternative YAML file when you want to swap in a different set of routines.

## OpenCR ros2_control hardware plugin

The `opencr_hardware` C++ package provides a `hardware_interface::SystemInterface` plugin that streams wheel velocities to/from the OpenCR USB bridge. The default `config/mini_ros2_control.urdf` already points the `<ros2_control>` block at `opencr_hardware/OpenCRSystem`; confirm that the board is flashed with the protocol (set `RUN_WHEEL_ID_TEST_ON_BOOT` to `false` once the wheel IDs/directions are known).

The serial settings now follow the same environment variables as the pytest (`OPENCR_SERIAL_PORT` and `OPENCR_SERIAL_BAUD`). Export the ones you need before launching, e.g.:

```bash
export OPENCR_SERIAL_PORT=/dev/ttyUSB0
export OPENCR_SERIAL_BAUD=230400
ros2 launch robot_main robot_main.launch.py
```

`controller_manager` and the controller spawners will automatically skip launching if the resolved serial device does not exist so the rest of the bringup (map server, localization, RViz, etc.) can continue on development machines without hardware attached. As soon as the device path appears again, re-run the launch command and the full ros2_control stack will be included.

Need to turn off the ros2_control stack entirely (for example when the board is busy or unplugged but the `/dev/ttyACM*` path still exists)? Export `OPENCR_DISABLE_ROS2_CONTROL=1` before launching. The `requires_device` entries in `config/robot_main.yaml` inspect this flag and skip the hardware-dependent controller manager and spawners whenever it is set.

Key parameters exposed in the URDF:

- `serial_port`: USB device (defaults to `/dev/ttyACM0`).
- `baud_rate`: must match the firmware (default `115200`).
- `status_timeout_sec`: warn if no status packet arrives for this window.
- `max_read_iterations`: bounds how many serial read bursts are processed per control loop.

Build the hardware plugin and launch normally:

```bash
colcon build --packages-select opencr_hardware --merge-install
source install/setup.bash
ros2 launch robot_main robot_main.launch.py
```

During runtime the plugin forwards every wheel command frame to the OpenCR and integrates the measured velocities that come back in `STATUS` packets so `ros2_control` receives both position and velocity states for the mecanum joints. The watchdog bit reported by the board is surfaced as a throttled warning whenever it trips, so you can catch transport stalls while teleoperating.

<!-- ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /ws/config/mini_0.urdf)" \
  -r /robot_description:=/controller_manager/robot_description


ros2 run controller_manager ros2_control_node \
  --ros-args --params-file /ws/config/ros2_controllers.yaml

ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner mecanum_controller -->
