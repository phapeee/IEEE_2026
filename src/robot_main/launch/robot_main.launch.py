"""Launch description for the robot_main global launcher."""

import math
import os
from pathlib import Path
from typing import Any, Dict, List

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    config = _load_robot_main_config(config_path)
    robot_description_override_path = config.get("robot_description_file")
    robot_description_source = str(robot_description_override_path or robot_description_file)
    global_namespace = _normalize_namespace(config.get("namespace", ""))
    robot_namespace_arg = global_namespace.strip("/") if global_namespace else ""
    xacro_command = [FindExecutable(name="xacro"), " ", robot_description_source]
    if robot_namespace_arg:
        xacro_command.extend([" ", f"robot_namespace:={robot_namespace_arg}"])
    opencr_serial_port = os.environ.get("OPENCR_SERIAL_PORT")
    opencr_serial_baud = os.environ.get("OPENCR_SERIAL_BAUD")
    if opencr_serial_port:
        xacro_command.extend([" ", f"opencr_serial_port:={opencr_serial_port}"])
    if opencr_serial_baud:
        xacro_command.extend([" ", f"opencr_baud_rate:={opencr_serial_baud}"])

    robot_description = ParameterValue(
        Command(xacro_command),
        value_type=str,
    )
    nodes: List[Node] = []

    if _component_enabled(config, "controller_manager"):
        controller_cfg = _component_config(config, "controller_manager")
        controller_parameters = controller_cfg.get("parameters", {})
        controller_params_files = _controller_manager_param_sources(controller_cfg, mecanum_config_path)
        cm_parameters = [{"robot_description": robot_description}, *controller_params_files]
        if controller_parameters:
            cm_parameters.append(controller_parameters)
        cm_namespace = _resolve_namespace(global_namespace, controller_cfg.get("namespace"))
        cm_parameters = _namespace_frame_ids(cm_parameters, global_namespace)
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=cm_parameters,
                namespace=cm_namespace,
                output=controller_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "joint_state_broadcaster_spawner"):
        js_cfg = _component_config(config, "joint_state_broadcaster_spawner")
        controller_name = js_cfg.get("controller_name", "joint_state_broadcaster")
        controller_manager_ns = js_cfg.get("controller_manager", "/controller_manager")
        js_namespace = _resolve_namespace(global_namespace, js_cfg.get("namespace"))
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "--controller-manager", controller_manager_ns],
                namespace=js_namespace,
                output=js_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "mecanum_controller_spawner"):
        mecanum_cfg = _component_config(config, "mecanum_controller_spawner")
        controller_name = mecanum_cfg.get("controller_name", "mecanum_controller")
        controller_manager_ns = mecanum_cfg.get("controller_manager", "/controller_manager")
        extra_args = mecanum_cfg.get("extra_arguments", [])
        mecanum_namespace = _resolve_namespace(global_namespace, mecanum_cfg.get("namespace"))
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "--controller-manager", controller_manager_ns, *extra_args],
                namespace=mecanum_namespace,
                output=mecanum_cfg.get("output", "screen"),
            )
        )
    if _component_enabled(config, "rocker_bogie_controller_spawner"):
        rocker_cfg = _component_config(config, "rocker_bogie_controller_spawner")
        controller_name = rocker_cfg.get("controller_name", "rocker_bogie_controller")
        controller_manager_ns = rocker_cfg.get("controller_manager", "/controller_manager")
        extra_args = rocker_cfg.get("extra_arguments", [])
        rocker_namespace = _resolve_namespace(global_namespace, rocker_cfg.get("namespace"))
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "--controller-manager", controller_manager_ns, *extra_args],
                namespace=rocker_namespace,
                output=rocker_cfg.get("output", "screen"),
            )
        )

    if _component_enabled(config, "robot_state_publisher"):
        rsp_cfg = _component_config(config, "robot_state_publisher")
        namespace = _resolve_namespace(global_namespace, rsp_cfg.get("namespace"))
        rsp_parameters = {"publish_robot_description": True}
        rsp_parameters.update(rsp_cfg.get("parameters", {}))
        robot_description_override = rsp_parameters.pop("robot_description", None)
        rsp_parameter_list: List[Dict[str, Any]] = [
            _build_robot_description_parameter(robot_description_override, robot_description)
        ]
        if rsp_parameters:
            rsp_parameter_list.append(rsp_parameters)
        rsp_parameter_list = _namespace_frame_ids(rsp_parameter_list, global_namespace)
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
        relay_parameters = _namespaced_cmd_vel_topics(relay_cfg.get("ros__parameters", {}), global_namespace)
        relay_namespace = _resolve_namespace(global_namespace, relay_cfg.get("namespace"))
        relay_parameter_list = _namespace_frame_ids([relay_parameters], global_namespace)
        nodes.append(
            Node(
                package="cmd_vel_relay",
                executable="cmd_vel_relay_node",
                name=relay_cfg.get("name", "cmd_vel_relay"),
                parameters=relay_parameter_list,
                namespace=relay_namespace,
                output=relay_cfg.get("output", "screen"),
            )
        )

    for component_name in _component_variant_names(config, "pointcloud_to_laserscan"):
        if _component_enabled(config, component_name):
            nodes.extend(_create_pointcloud_to_laserscan_nodes(config, global_namespace, component_name=component_name))

    if _component_enabled(config, "pointcloud_concatenate"):
        nodes.extend(_create_pointcloud_concatenate_nodes(config, global_namespace))

    if _component_enabled(config, "pointcloud_filter"):
        nodes.extend(_create_pointcloud_filter_nodes(config, global_namespace))

    if _component_enabled(config, "robot_localization"):
        nodes.extend(_create_robot_localization_nodes(config, global_namespace))

    if _component_enabled(config, "wit_imu_driver"):
        nodes.extend(_create_wit_imu_driver_nodes(config, global_namespace))

    if _component_enabled(config, "nav2_amcl"):
        nodes.extend(_create_nav2_amcl_nodes(config, global_namespace))

    if _component_enabled(config, "nav2_map_server"):
        nodes.extend(_create_nav2_map_server_nodes(config, global_namespace))

    if _component_enabled(config, "nav2_stack"):
        nodes.extend(_create_nav2_stack_nodes(config, global_namespace))

    if _component_enabled(config, "holonomic_pi_controller"):
        nodes.extend(_create_holonomic_pi_controller_nodes(config, global_namespace))

    for component_name in _component_variant_names(config, "nav2_lifecycle_manager"):
        if _component_enabled(config, component_name):
            nodes.extend(_create_nav2_lifecycle_manager_nodes(config, global_namespace, component_name=component_name))

    if _component_enabled(config, "laser_scan_merger"):
        nodes.extend(_create_laser_scan_merger_nodes(config, global_namespace))

    if _component_enabled(config, "imu_calibration"):
        nodes.extend(_create_imu_calibration_nodes(config, global_namespace))

    if _component_enabled(config, "initial_pose_publisher"):
        nodes.extend(_create_initial_pose_publisher_nodes(config, global_namespace))

    if _component_enabled(config, "gpio_button_event"):
        nodes.extend(_create_gpio_button_event_nodes(config, global_namespace))

    if _component_enabled(config, "limit_switch_calibration"):
        nodes.extend(_create_limit_switch_calibration_nodes(config, global_namespace))

    if _component_enabled(config, "waypoint_state_machine"):
        nodes.extend(_create_waypoint_state_machine_nodes(config, global_namespace))

    if _component_enabled(config, "uwb_triangulaion"):
        nodes.extend(_create_uwb_triangulaion_nodes(config, global_namespace))

    return nodes


def _create_gpio_button_event_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "gpio_button_event")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "gpio_button_event")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "gpio_button_event"),
            executable=node_cfg.get("executable", "gpio_button_event_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_limit_switch_calibration_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "limit_switch_calibration")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "limit_switch_calibration")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "limit_switch_calibration"),
            executable=node_cfg.get("executable", "calibration_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_waypoint_state_machine_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "waypoint_state_machine")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "waypoint_state_machine")
    parameter_entries: List[Any] = []
    if params_file:
        loaded = _load_parameters_from_file(params_file, node_name)
        if loaded:
            parameter_entries.append(loaded)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "smacc_button_nav"),
            executable=node_cfg.get("executable", "button_waypoint_sm"),
            name=node_name,
            # SIMPLE gdb: you will get a (gdb) prompt
            # prefix='gdb --args',
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_pointcloud_to_laserscan_nodes(config: Dict[str, Any], global_namespace: str, component_name: str = "pointcloud_to_laserscan") -> List[Node]:
    node_cfg = _component_config(config, component_name)
    params_file = node_cfg.get("params_file", _discover_default_config("pointcloud_to_laserscan.yaml"))
    cloud_override = node_cfg.get("cloud_topic", "")
    scan_override = node_cfg.get("scan_topic", "")
    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))

    topics = _load_topics(params_file)
    cloud_topic = cloud_override or topics.get("cloud_in", "/pointcloud")
    scan_topic = scan_override or topics.get("scan", "/scan")
    cloud_topic = _namespaced_topic(global_namespace, cloud_topic)
    scan_topic = _namespaced_topic(global_namespace, scan_topic)

    node_name = node_cfg.get("name", f"{component_name}_node")
    parameter_entries: List[Any] = []
    loaded_parameters = _load_parameters_from_file(params_file, node_name)
    if loaded_parameters:
        parameter_entries.append(loaded_parameters)
    inline_parameters = node_cfg.get("ros__parameters", {})
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    parameter_list = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name=node_name,
            parameters=parameter_list,
            remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _controller_manager_param_sources(controller_cfg: Dict[str, Any], default_config: str) -> List[str]:
    params_entry = controller_cfg.get("params_file")
    if isinstance(params_entry, list):
        return params_entry or [default_config]
    if isinstance(params_entry, str) and params_entry.strip():
        return [params_entry]
    return [default_config]


def _create_pointcloud_concatenate_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "pointcloud_concatenate")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_parameters: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_cfg.get("name", "pointcloud_concatenate"))
        if loaded_parameters:
            node_parameters.append(loaded_parameters)
    if inline_parameters:
        node_parameters.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))

    node_parameters = _namespace_frame_ids(node_parameters, global_namespace)
    
    return [
        Node(
            package=node_cfg.get("package", "pointcloud_concatenate"),
            executable=node_cfg.get("executable", "pointcloud_concatenate_node"),
            name=node_cfg.get("name", "pointcloud_concatenate"),
            parameters=node_parameters,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_pointcloud_filter_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "pointcloud_filter")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_parameters: List[Any] = []
    node_name = node_cfg.get("name", "pointcloud_filter")
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            node_parameters.append(loaded_parameters)
    if inline_parameters:
        node_parameters.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    node_parameters = _namespace_frame_ids(node_parameters, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "pointcloud_filter"),
            executable=node_cfg.get("executable", "pointcloud_filter_node"),
            name=node_name,
            parameters=node_parameters,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_robot_localization_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "robot_localization")
    params_file = node_cfg.get("params_file", _discover_default_config("robot_localization.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "ekf_filter_node")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "robot_localization"),
            executable=node_cfg.get("executable", "ekf_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_nav2_amcl_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "nav2_amcl")
    params_file = node_cfg.get("params_file", _discover_default_config("nav2_amcl.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "amcl")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "nav2_amcl"),
            executable=node_cfg.get("executable", "amcl"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_nav2_map_server_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "nav2_map_server")
    params_file = node_cfg.get("params_file", _discover_default_config("nav2_map_server.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "map_server")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    # Map server publishes global /map so keep it outside robot namespace unless explicitly overridden.
    component_namespace = _normalize_namespace(node_cfg.get("namespace"))
    node_namespace = component_namespace

    return [
        Node(
            package=node_cfg.get("package", "nav2_map_server"),
            executable=node_cfg.get("executable", "map_server"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


_NAV2_STACK_NODE_DEFAULTS: Dict[str, Dict[str, str]] = {
    "controller_server": {"package": "nav2_controller", "executable": "controller_server"},
    "planner_server": {"package": "nav2_planner", "executable": "planner_server"},
    "smoother_server": {"package": "nav2_smoother", "executable": "smoother_server"},
    "behavior_server": {"package": "nav2_behaviors", "executable": "behavior_server"},
    "bt_navigator": {"package": "nav2_bt_navigator", "executable": "bt_navigator"},
    "waypoint_follower": {"package": "nav2_waypoint_follower", "executable": "waypoint_follower"},
    "velocity_smoother": {"package": "nav2_velocity_smoother", "executable": "velocity_smoother"},
}


def _create_nav2_stack_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "nav2_stack")
    params_file = node_cfg.get("params_file", _discover_default_config("nav2_params.yaml"))
    nodes_to_launch = node_cfg.get("nodes") or list(_NAV2_STACK_NODE_DEFAULTS.keys())
    overrides = node_cfg.get("node_overrides", {})
    component_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    default_output = node_cfg.get("output", "screen")

    nav2_nodes: List[Node] = []
    for nav2_node in nodes_to_launch:
        defaults = _NAV2_STACK_NODE_DEFAULTS.get(nav2_node)
        if not defaults:
            continue
        override_cfg = overrides.get(nav2_node, {})
        package = override_cfg.get("package", defaults["package"])
        executable = override_cfg.get("executable", defaults["executable"])
        node_name = override_cfg.get("name", nav2_node)
        node_namespace = _resolve_namespace(component_namespace, override_cfg.get("namespace"))
        node_parameters: List[Any] = []
        if params_file:
            loaded_parameters = _load_parameters_from_file(params_file, nav2_node)
            if loaded_parameters:
                node_parameters.append(loaded_parameters)
        inline_params = override_cfg.get("ros__parameters")
        if isinstance(inline_params, dict) and inline_params:
            node_parameters.append(inline_params)
        node_parameters = _namespace_frame_ids(node_parameters, global_namespace)

        remappings = override_cfg.get("remappings", [])
        if isinstance(remappings, dict):
            remappings = list(remappings.items())
        elif not isinstance(remappings, list):
            remappings = []
        extra_arguments = override_cfg.get("extra_arguments", [])
        if not isinstance(extra_arguments, list):
            extra_arguments = [extra_arguments]

        nav2_nodes.append(
            Node(
                package=package,
                executable=executable,
                name=node_name,
                namespace=node_namespace,
                parameters=node_parameters,
                remappings=remappings,
                arguments=extra_arguments,
                output=override_cfg.get("output", default_output),
            )
        )

    return nav2_nodes


def _create_holonomic_pi_controller_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "holonomic_pi_controller")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    parameter_entries: List[Any] = []
    node_name = node_cfg.get("name", "holonomic_pi_controller")
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "holonomic_pi_controller"),
            executable=node_cfg.get("executable", "holonomic_pi_controller_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_nav2_lifecycle_manager_nodes(
    config: Dict[str, Any], global_namespace: str, component_name: str = "nav2_lifecycle_manager"
) -> List[Node]:
    node_cfg = _component_config(config, component_name)
    params_file = node_cfg.get("params_file", _discover_default_config("lifecycle_localization.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "lifecycle_manager_localization")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    component_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_lifecycle_node_names(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "nav2_lifecycle_manager"),
            executable=node_cfg.get("executable", "lifecycle_manager"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=component_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_initial_pose_publisher_nodes(config: Dict[str, Any], global_namespace: str) -> List[Any]:
    """Publish a single initial pose message on the (namespaced) initialpose topic."""
    node_cfg = _component_config(config, "initial_pose_publisher")
    params_file = node_cfg.get("params_file")
    loaded_cfg: Dict[str, Any] = {}
    if params_file:
        loaded_cfg = _load_parameters_from_file(params_file, node_cfg.get("name", "initial_pose_publisher"))

    def _param(key: str, default: Any) -> Any:
        if key in node_cfg and node_cfg.get(key) is not None:
            return node_cfg.get(key)
        if key in loaded_cfg and loaded_cfg.get(key) is not None:
            return loaded_cfg.get(key)
        return default

    x = float(_param("x", 0.0))
    y = float(_param("y", 0.0))
    yaw = float(_param("yaw", 0.0))
    frame_id = _namespaced_frame_id(global_namespace, _param("frame_id", "map"))
    topic = _namespaced_topic(global_namespace, _param("topic", "initialpose"))
    delay_sec = float(_param("delay_sec", 1.0))
    tf_publish = bool(_param("tf_publish", False))
    tf_parent_frame = _namespaced_frame_id(
        global_namespace, _param("tf_parent_frame", frame_id))
    tf_child_frame = _namespaced_frame_id(
        global_namespace, _param("tf_child_frame", "base_link"))

    half_yaw = yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    covariance_str = ", ".join(str(v) for v in covariance)
    pose_msg = (
        f"{{header: {{frame_id: '{frame_id}'}}, "
        f"pose: {{pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{z: {qz}, w: {qw}}}}}, "
        f"covariance: [{covariance_str}]}}}}"
    )

    initial_pose_script = Path("/ws/scripts/initial_pose_publisher.py")
    publisher_cmd = [
        "python3",
        str(initial_pose_script),
        "--topic",
        topic,
        "--frame-id",
        frame_id,
        "--x",
        str(x),
        "--y",
        str(y),
        "--yaw",
        str(yaw),
        "--delay",
        str(delay_sec),
    ]

    actions: List[Any] = [
        ExecuteProcess(
            cmd=publisher_cmd,
            output=node_cfg.get("output", "screen"),
        )
    ]

    if tf_publish:
        static_tf_cmd = [
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            str(x),
            str(y),
            "0.0",
            "0.0",
            "0.0",
            str(qz),
            str(qw),
            tf_parent_frame,
            tf_child_frame,
        ]
        actions.append(
            ExecuteProcess(
                cmd=static_tf_cmd,
                output=node_cfg.get("output", "screen"),
            )
        )

    return actions


def _create_laser_scan_merger_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "laser_scan_merger")
    params_file = node_cfg.get("params_file", _discover_default_config("laser_scan_merger.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "laser_scan_merger_node")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "laser_scan_merger"),
            executable=node_cfg.get("executable", "laser_scan_merger_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_uwb_triangulaion_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "uwb_triangulaion")
    params_file = node_cfg.get("params_file", _discover_default_config("uwb_config.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    base_parameters: Dict[str, Any] = {}
    if params_file:
        base_parameters["config_file"] = params_file

    parameter_entries: List[Any] = []
    if base_parameters:
        parameter_entries.append(base_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "uwb_triangulaion"),
            executable=node_cfg.get("executable", "uwb_triangulaion_node"),
            name=node_cfg.get("name", "uwb_triangulaion_node"),
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_imu_calibration_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "imu_calibration")
    params_file = node_cfg.get("params_file", _discover_default_config("imu_calibration.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "imu_calibration_node")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "imu_calibration"),
            executable=node_cfg.get("executable", "imu_calibration_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_wit_imu_driver_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "wit_imu_driver")
    params_file = node_cfg.get("params_file", _discover_default_config("wit_imu_driver.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "wit_imu_node")
    parameter_entries: List[Any] = []
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "wit_imu_driver"),
            executable=node_cfg.get("executable", "wit_imu_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _build_robot_description_parameter(override: Any, default_value: ParameterValue) -> Dict[str, Any]:
    if override is None:
        return {"robot_description": default_value}
    command_path = _extract_xacro_target(override) if isinstance(override, str) else None
    if command_path:
        return {
            "robot_description": ParameterValue(
                Command([FindExecutable(name="xacro"), " ", command_path]),
                value_type=str,
            )
        }
    return {"robot_description": override}


def _extract_xacro_target(value: str) -> str | None:
    stripped = value.strip()
    if not (stripped.startswith("$(xacro") and stripped.endswith(")")):
        return None
    inner = stripped[len("$(xacro") : -1].strip()
    return inner or None


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


def _component_requirements_met(component_name: str, component_cfg: Dict[str, Any]) -> bool:
    requirement = component_cfg.get("requires_device")
    disabled_reason = _requirement_disabled_reason(requirement)
    if disabled_reason:
        print(f"[robot_main.launch] Skipping '{component_name}': {disabled_reason}")
        return False
    missing_devices = _missing_required_devices(requirement)
    if missing_devices:
        missing_str = ", ".join(missing_devices)
        print(
            f"[robot_main.launch] Skipping '{component_name}' because the following device(s) are missing: {missing_str}"
        )
        return False
    return True


def _requirement_disabled_reason(requirement: Any) -> str | None:
    if not requirement:
        return None
    if isinstance(requirement, dict):
        disable_env = requirement.get("disable_env")
        if disable_env and _env_var_truthy(disable_env):
            return f"environment variable {disable_env} disables this component"
    return None


def _missing_required_devices(requirement: Any) -> List[str]:
    if not requirement:
        return []
    required_paths = _resolve_required_device_entries(requirement)
    missing = [str(path) for path in required_paths if not path.exists()]
    return missing


def _resolve_required_device_entries(requirement: Any) -> List[Path]:
    if requirement is None:
        return []
    if isinstance(requirement, str):
        return [Path(os.path.expandvars(os.path.expanduser(requirement)))]
    if isinstance(requirement, dict):
        paths: List[str] = []
        explicit_path = requirement.get("path")
        if explicit_path:
            paths.append(str(explicit_path))
        env_name = requirement.get("env")
        if env_name:
            env_value = os.environ.get(env_name)
            if env_value:
                paths.append(env_value)
        default_value = requirement.get("default")
        if default_value:
            paths.append(str(default_value))
        additional = requirement.get("paths")
        if isinstance(additional, list):
            paths.extend(str(item) for item in additional if item)
        resolved: List[Path] = []
        for value in paths:
            expanded = os.path.expandvars(os.path.expanduser(value))
            resolved.append(Path(expanded))
        return resolved
    if isinstance(requirement, list):
        resolved: List[Path] = []
        for item in requirement:
            resolved.extend(_resolve_required_device_entries(item))
        return resolved
    return []


def _env_var_truthy(name: str) -> bool:
    value = os.environ.get(name)
    if value is None:
        return False
    return value not in ("", "0", "false", "False")

def _component_variant_names(config: Dict[str, Any], base_name: str) -> List[str]:
    components = config.get("components", {})
    suffix = f"{base_name}_"
    return [name for name in components.keys() if name == base_name or name.startswith(suffix)]


def _component_enabled(config: Dict[str, Any], name: str) -> bool:
    components = config.get("components", {})
    if name not in components:
        return False
    entry = _component_config(config, name)
    if "enabled" not in entry:
        return _component_requirements_met(name, entry)
    if not bool(entry.get("enabled")):
        return False
    return _component_requirements_met(name, entry)


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


def _load_yaml_file(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    return data if isinstance(data, dict) else {}


def _load_parameters_from_file(path_str: str, node_name: str) -> Dict[str, Any]:
    path = Path(path_str)
    data = _load_yaml_file(path)
    if not data:
        return {}
    candidates = [
        node_name,
        f"/{node_name}",
        "/**",
    ]
    entry: Dict[str, Any] = {}
    for key in candidates:
        maybe_entry = data.get(key)
        if isinstance(maybe_entry, dict):
            entry = maybe_entry
            break
    if not entry:
        # Fall back to the first dict entry, if any.
        for maybe_entry in data.values():
            if isinstance(maybe_entry, dict):
                entry = maybe_entry
                break
    if not entry:
        return {}
    ros_parameters = entry.get("ros__parameters")
    if isinstance(ros_parameters, dict):
        return ros_parameters
    return entry


def _namespace_frame_ids(parameters: List[Any], global_namespace: str) -> List[Any]:
    if not global_namespace or not parameters:
        return parameters
    return [_apply_frame_id_namespace(entry, global_namespace) for entry in parameters]


def _namespace_lifecycle_node_names(parameters: List[Any], global_namespace: str) -> List[Any]:
    if not global_namespace or not parameters:
        return parameters
    namespaced: List[Any] = []
    for entry in parameters:
        if isinstance(entry, dict):
            node_names = entry.get("node_names")
            if isinstance(node_names, list):
                updated_entry = dict(entry)
                updated_entry["node_names"] = [
                    _force_namespaced_topic(global_namespace, name) if isinstance(name, str) else name
                    for name in node_names
                ]
                namespaced.append(updated_entry)
                continue
        namespaced.append(entry)
    return namespaced


def _apply_frame_id_namespace(entry: Any, global_namespace: str) -> Any:
    if isinstance(entry, dict):
        updated: Dict[Any, Any] = {}
        for key, value in entry.items():
            if key == "frame_id":
                updated[key] = _namespaced_frame_id(global_namespace, value)
            elif key == "target_frame":
                updated[key] = _namespaced_target_frame(global_namespace, value)
            elif key == "global_frame_id":
                updated[key] = value
            elif _is_frame_reference_key(key):
                updated[key] = _namespaced_frame_id(global_namespace, value)
            elif _is_topic_key(key):
                updated[key] = _namespaced_topic(global_namespace, value)
            else:
                updated[key] = _apply_frame_id_namespace(value, global_namespace)
        return updated
    if isinstance(entry, list):
        return [_apply_frame_id_namespace(item, global_namespace) for item in entry]
    return entry


def _namespaced_frame_id(global_namespace: str, frame_id_value: Any) -> Any:
    if not isinstance(frame_id_value, str):
        return frame_id_value
    candidate = _clean_frame_reference(frame_id_value)
    if not global_namespace:
        return candidate
    namespace_without_slash = global_namespace.strip("/")
    if not candidate:
        return namespace_without_slash
    if candidate == namespace_without_slash or candidate.startswith(f"{namespace_without_slash}/"):
        return candidate
    return f"{namespace_without_slash}/{candidate}"


def _namespaced_target_frame(global_namespace: str, target_value: Any) -> Any:
    if not isinstance(target_value, str):
        return target_value
    candidate = _clean_frame_reference(target_value)
    if not global_namespace:
        return candidate
    namespace_without_slash = global_namespace.strip("/")
    if not candidate:
        return namespace_without_slash
    if candidate == namespace_without_slash or candidate.startswith(f"{namespace_without_slash}/"):
        return candidate
    return f"{namespace_without_slash}/{candidate}"


def _clean_frame_reference(value: str) -> str:
    stripped = value.strip()
    return stripped.strip("/") if stripped else ""


def _namespaced_cmd_vel_topics(relay_parameters: Any, global_namespace: str) -> Any:
    """Ensure cmd_vel_relay topic parameters get the global namespace prefix."""
    if not isinstance(relay_parameters, dict):
        return relay_parameters
    if not global_namespace:
        return relay_parameters
    namespaced = dict(relay_parameters)
    for topic_key in ("input_topic", "output_topic"):
        if topic_key in namespaced:
            namespaced[topic_key] = _force_namespaced_topic(global_namespace, namespaced[topic_key])
    return namespaced


def _force_namespaced_topic(global_namespace: str, topic_value: Any) -> Any:
    """Prefix a topic with the global namespace even if it was provided as absolute."""
    if not isinstance(topic_value, str):
        return topic_value
    topic = topic_value.strip()
    if not topic:
        return topic
    base_ns = _normalize_namespace(global_namespace)
    if not base_ns:
        return topic
    if topic == base_ns or topic.startswith(f"{base_ns}/"):
        return topic
    if topic.startswith("/"):
        topic = topic[1:]
    base = base_ns.rstrip("/")
    return f"{base}/{topic}" if topic else base


def _is_topic_key(key: Any) -> bool:
    if not isinstance(key, str):
        return False
    lowered = key.lower()
    if lowered.endswith("_topic") or lowered == "topic":
        return True
    for prefix in ("imu", "odom", "pose", "twist", "accel", "gps", "wheel", "vo", "visual_odom", "range"):
        if lowered.startswith(prefix) and lowered[len(prefix) :].isdigit():
            return True
    return False


def _is_frame_reference_key(key: Any) -> bool:
    if not isinstance(key, str):
        return False
    lowered = key.lower()
    return lowered.endswith("_frame") or lowered.endswith("_frame_id")


def _namespaced_topic(global_namespace: str, topic_value: Any) -> Any:
    if not isinstance(topic_value, str):
        return topic_value
    topic = topic_value.strip()
    if not topic:
        return topic
    if topic.startswith("/"):
        return topic
    if not global_namespace:
        return topic
    base = global_namespace.rstrip("/")
    return f"{base}/{topic}"


def _normalize_namespace(value: Any) -> str:
    if not isinstance(value, str):
        return ""
    stripped = value.strip()
    if not stripped or stripped == "/":
        return ""
    if not stripped.startswith("/"):
        stripped = "/" + stripped
    while len(stripped) > 1 and stripped.endswith("/"):
        stripped = stripped[:-1]
    return stripped


def _resolve_namespace(global_namespace: str, component_namespace: Any) -> str:
    component_ns = _normalize_namespace(component_namespace)
    if not global_namespace:
        return component_ns
    if not component_ns:
        return global_namespace
    # component_ns already starts with '/', so skip duplicate slash.
    return f"{global_namespace}{component_ns}"
