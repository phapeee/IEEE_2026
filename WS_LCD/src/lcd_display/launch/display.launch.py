from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("st7789_ros_wrapper"),
                "config",
                "display.yaml",
            ]),
            description="Path to the display node parameter YAML file.",
        ),
        Node(
            package="st7789_ros_wrapper",
            executable="st7789_display_node",
            name="st7789_display_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
