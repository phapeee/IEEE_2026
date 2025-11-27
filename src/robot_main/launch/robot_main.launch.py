"""Launch description for the robot_main global launcher."""

from pathlib import Path
from typing import Any, Dict, List

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_COMPONENT_DEFAULTS: Dict[str, bool] = {
    "pointcloud_to_laserscan": False,
}


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

    return LaunchDescription(
        [
            config_arg,
            mecanum_config_arg,
            robot_description_arg,
            OpaqueFunction(function=_launch_components),
        ]
    )


def _launch_components(context, *_: Any) -> List[Node]:
    config_path = Path(context.perform_substitution(LaunchConfiguration("robot_main_config")))
    mecanum_config_path = context.perform_substitution(LaunchConfiguration("mecanum_controller_config"))
    robot_description_file = context.perform_substitution(LaunchConfiguration("robot_description_file"))

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_description_file]),
        value_type=str,
    )

    config = _load_robot_main_config(config_path)
    nodes: List[Node] = []

    if _component_enabled(config, "controller_manager"):
        controller_cfg = _component_config(config, "controller_manager")
        controller_parameters = controller_cfg.get("parameters", {})
        cm_parameters = [{"robot_description": robot_description}, mecanum_config_path]
        if controller_parameters:
            cm_parameters.append(controller_parameters)
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=cm_parameters,
                output=controller_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "joint_state_broadcaster_spawner"):
        js_cfg = _component_config(config, "joint_state_broadcaster_spawner")
        controller_name = js_cfg.get("controller_name", "joint_state_broadcaster")
        controller_manager_ns = js_cfg.get("controller_manager", "/controller_manager")
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "--controller-manager", controller_manager_ns],
                output=js_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "mecanum_controller_spawner"):
        mecanum_cfg = _component_config(config, "mecanum_controller_spawner")
        controller_name = mecanum_cfg.get("controller_name", "mecanum_controller")
        controller_manager_ns = mecanum_cfg.get("controller_manager", "/controller_manager")
        extra_args = mecanum_cfg.get("extra_arguments", [])
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "--controller-manager", controller_manager_ns, *extra_args],
                output=mecanum_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "robot_state_publisher"):
        rsp_cfg = _component_config(config, "robot_state_publisher")
        rsp_parameters = rsp_cfg.get("parameters", {"publish_robot_description": True})
        namespace = rsp_cfg.get("namespace", "controller_manager")
        rsp_parameter_list = [{"robot_description": robot_description}]
        if rsp_parameters:
            rsp_parameter_list.append(rsp_parameters)
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=rsp_parameter_list,
                namespace=namespace,
                output=rsp_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "cmd_vel_relay"):
        relay_cfg = _component_config(config, "cmd_vel_relay")
        relay_parameters = relay_cfg.get("ros__parameters", {})
        nodes.append(
            Node(
                package="cmd_vel_relay",
                executable="cmd_vel_relay_node",
                name=relay_cfg.get("name", "cmd_vel_relay"),
                parameters=[relay_parameters],
                output=relay_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "pointcloud_to_laserscan"):
        nodes.extend(_create_pointcloud_to_laserscan_nodes(config))

    return nodes


def _create_pointcloud_to_laserscan_nodes(config: Dict[str, Any]) -> List[Node]:
    node_cfg = _component_config(config, "pointcloud_to_laserscan")
    params_file = node_cfg.get("params_file", _discover_default_config("pointcloud_to_laserscan.yaml"))
    cloud_override = node_cfg.get("cloud_topic", "")
    scan_override = node_cfg.get("scan_topic", "")

    topics = _load_topics(params_file)
    cloud_topic = cloud_override or topics.get("cloud_in", "/pointcloud")
    scan_topic = scan_override or topics.get("scan", "/scan")

    return [
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name=node_cfg.get("name", "pointcloud_to_laserscan_node"),
            parameters=[params_file],
            remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
            output=node_cfg.get("output", "screen"),
        )
    ]


def _discover_default_config(filename: str) -> str:
    """Search upward from this file for a config directory containing filename."""
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "config" / filename
        if candidate.exists():
            return str(candidate)
    # Fallback to a relative config/filename inside the current working directory.
    return str(Path.cwd() / "config" / filename)


def _load_robot_main_config(config_path: Path) -> Dict[str, Any]:
    if not config_path.exists():
        return {}
    with open(config_path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    return data


def _component_config(config: Dict[str, Any], name: str) -> Dict[str, Any]:
    components = config.get("components", {})
    entry = components.get(name, {})
    if isinstance(entry, bool):
        return {"enabled": entry}
    return entry or {}


def _component_enabled(config: Dict[str, Any], name: str) -> bool:
    entry = _component_config(config, name)
    default_state = _COMPONENT_DEFAULTS.get(name, True)
    return entry.get("enabled", default_state)


def _load_topics(config_path: str) -> Dict[str, Any]:
    path = Path(config_path)
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    node = data.get("pointcloud_to_laserscan_node", {})
    params = node.get("ros__parameters", {})
    topics = params.get("topics", {})
    return topics if isinstance(topics, dict) else {}
