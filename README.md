# ROS 2 Workspace

This workspace contains the `diff_drive_controller` package and the `robot_main` launcher package.

## Launching the system

After building the workspace and sourcing `install/setup.bash`, run:

```bash
ros2 launch robot_main robot_main.launch.py
```

The launch file now reads every node/component definition from `config/robot_main.yaml`. Each entry under the top-level `components` block declares whether the component is enabled plus any node-specific arguments. Leave the `robot_main_config` argument unset to use this default file or point it to an alternative YAML when you want different combinations of components.

```yaml
components:
  controller_manager:
    enabled: true
  joint_state_broadcaster_spawner:
    enabled: true
  mecanum_controller_spawner:
    enabled: true
  robot_state_publisher:
    enabled: true
  cmd_vel_relay:
    enabled: true
    ros__parameters:
      input_topic: "/cmd_vel"
      output_topic: "/mecanum_controller/reference"
  pointcloud_to_laserscan:
    enabled: false
```

Set `enabled` to `false` to keep a component out of the launch description. Optional fields let you override names, controller-manager namespaces, parameter overrides, or remappings on a per-component basis.

## Sending velocity commands

Launch a keyboard teleop node (for example `teleop_twist_keyboard`) in another terminal to publish `geometry_msgs/Twist` messages to `/cmd_vel`. The `cmd_vel_relay` section in `config/robot_main.yaml` sets the relay node name, enable flag, and the `ros__parameters` block that defines the input/output topics (default `/cmd_vel` → `/mecanum_controller/reference`) plus the `frame_id` stamped on the outgoing `geometry_msgs/TwistStamped` messages:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The remaining controller manager nodes, spawners, and the `robot_state_publisher` are now launched automatically through `robot_main.launch.py`, so you can configure and enable them entirely via `robot_main.yaml` instead of running manual `ros2 run` commands.

## UWB triangulation node

The `uwb_triangulaion` package contains a C++ node that consumes `uwb_msgs/msg/IntFloatArrayStamped` packets and publishes the estimated pose as `geometry_msgs/PoseWithCovarianceStamped`. The node automatically mirrors the namespace of the configured input topic (e.g. `/uwb/ranges` → `/uwb/uwb_pose` or `/ranges` → `/uwb_pose`) and forwards the incoming header.

Launch it after building and sourcing the workspace:

```bash
ros2 run uwb_triangulaion uwb_triangulaion_node \
  --ros-args -p config_file:=/ws/config/uwb_config.yaml
```

The configuration file (`config/uwb_config.yaml`) defines the input topic, anchor IDs with their positions, pose covariance (36 entries), and the sign constraints used to disambiguate solutions whenever only two anchors are available. Update this file to match the layout of your anchors and supply an empirically determined covariance that gets stamped into the outgoing pose.

### Covariance estimation helper

To help tune the pose covariance empirically, the same package ships a `uwb_covariance_estimator_node`. It subscribes to the pose output of the triangulation node, collects `sample_count` poses, computes the covariance of X/Y, and overwrites the `pose_covariance` block inside `config/uwb_config.yaml`.

```bash
ros2 run uwb_triangulaion uwb_covariance_estimator_node \
  --ros-args -p config_file:=/ws/config/uwb_config.yaml
```

The `covariance_estimator` section in the config file declares the pose topic to monitor (`input_topic`), the number of samples to use (`sample_count`), and whether raw samples should be exported as CSV (`export_data`). When `export_data` is `true`, the estimator writes `<config_stem>_samples.csv` next to the YAML file so you can visualize the distribution offline. Restart the estimator node whenever you want to refresh the covariance with a new dataset.

### Calibration launch file

To jointly run the triangulation and covariance estimator nodes during a calibration session, use the provided launch file:

```bash
ros2 launch uwb_triangulaion uwb_calibration.launch.py \
  config_file:=/ws/config/uwb_config.yaml
```

Override the `config_file` argument if you store the YAML elsewhere. The launch file keeps both nodes alive so you can gather the desired number of samples and have the covariance automatically written back into the config.

### Offline data plotting

For quick visualization of the exported CSV data, run the standalone helper script (no ROS node needed):

```bash
python3 scripts/data_plot.py --csv config/uwb_config_samples.csv --headers x y
```

Each requested header produces a histogram with an overlaid normal distribution fit plus basic stats (count, mean, std, variance) embedded on the chart, making it easy to inspect how well your samples cluster before finalizing the covariance.

### pointcloud_to_laserscan helper

Update `config/pointcloud_to_laserscan.yaml` with parameters such as `min_height`, `max_height`, `angle_*`, `range_*`, `scan_time`, `target_frame`, and the nested `topics` block (holding `cloud_in`/`scan`). Enable the `pointcloud_to_laserscan` component inside `config/robot_main.yaml` to launch the conversion node alongside the rest of the system. You can also pin a dedicated `params_file`, `cloud_topic`, or `scan_topic` override inside that component block when you want to diverge from the defaults baked into the pointcloud config file.
