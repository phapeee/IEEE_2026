# ROS 2 Workspace

This workspace contains the `diff_drive_controller` package and the `robot_main` launcher package.

## Launching the system

After building the workspace and sourcing `install/setup.bash`, run:

```bash
ros2 launch robot_main robot_main.launch.py
```

Omit the `robot_main_config` argument if you want to rely on the default configuration auto-discovery.

## Sending velocity commands

Launch a keyboard teleop node (for example `teleop_twist_keyboard`) in another terminal to publish `geometry_msgs/Twist` messages to `/cmd_vel`. The `cmd_vel_relay` package consumes these messages and republishes them as `geometry_msgs/TwistStamped` on the topic configured in `config/robot_main.yaml` (defaults to `/controller_manager/mecanum_controller/reference`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /ws/config/mini.urdf)" \
  -r /robot_description:=/controller_manager/robot_description


ros2 run controller_manager ros2_control_node \
  --ros-args --params-file /ws/config/ros2_controllers.yaml

ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner mecanum_controller
```
