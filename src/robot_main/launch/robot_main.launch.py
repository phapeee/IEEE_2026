"""Launch description for the robot_main global launcher."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _discover_default_config(filename: str) -> str:
    """Search upward from this file for a config directory containing filename."""
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "config" / filename
        if candidate.exists():
            return str(candidate)
    # Fallback to a relative config/filename inside the current working directory.
    return str(Path.cwd() / "config" / filename)


def generate_launch_description() -> LaunchDescription:
    default_config_path = _discover_default_config("robot_main.yaml")
    default_mecanum_config_path = _discover_default_config("ros2_controllers.yaml")
    default_urdf_path = _discover_default_config("mini.urdf")

    config_arg = DeclareLaunchArgument(
        "robot_main_config",
        default_value=default_config_path,
        description="Absolute path to the YAML configuration file that holds all robot parameters.",
    )

    mecanum_config_arg = DeclareLaunchArgument(
        "mecanum_controller_config",
        default_value=default_mecanum_config_path,
        description="Absolute path to the mecanum controller YAML file.",
    )

    robot_description_arg = DeclareLaunchArgument(
        "robot_description_file",
        default_value=default_urdf_path,
        description="Absolute path to the robot URDF/XACRO file.",
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", LaunchConfiguration("robot_description_file")]),
        value_type=str,
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            LaunchConfiguration("mecanum_controller_config"),
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"publish_robot_description": True},
        ],
        namespace="controller_manager",
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller", "--controller-manager", "/controller_manager"],
    )

    cmd_vel_relay_node = Node(
        package="cmd_vel_relay",
        executable="cmd_vel_relay_node",
        name="cmd_vel_relay",
        parameters=[LaunchConfiguration("robot_main_config")],
        output="screen",
    )

    return LaunchDescription(
        [
            config_arg,
            mecanum_config_arg,
            robot_description_arg,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            mecanum_controller_spawner,
            robot_state_publisher_node,
            cmd_vel_relay_node,
        ]
    )
