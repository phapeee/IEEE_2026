"""Launch description for the robot_main global launcher."""

import math
import os
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_CONFIG_BASE_DIR: Path | None = None


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


def _launch_components(context, *_: Any) -> List[Any]:
    global _CONFIG_BASE_DIR
    config_path = Path(context.perform_substitution(LaunchConfiguration("robot_main_config")))
    if not config_path.is_absolute():
        config_path = Path.cwd() / config_path
    _CONFIG_BASE_DIR = config_path.parent
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
    nodes: List[Any] = []

    if _component_enabled(config, "controller_manager"):
        controller_cfg = _component_config(config, "controller_manager")
        controller_parameters = controller_cfg.get("parameters", {})
        controller_params_files = _controller_manager_param_sources(controller_cfg, mecanum_config_path)
        cm_parameters = [{"robot_description": robot_description}, *controller_params_files]
        if controller_parameters:
            cm_parameters.append(controller_parameters)
        cm_namespace = _resolve_namespace(global_namespace, controller_cfg.get("namespace"))
        cm_parameters = _namespace_frame_ids(cm_parameters, global_namespace)
        cm_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=cm_parameters,
            namespace=cm_namespace,
            output=controller_cfg.get("output", "screen"),
        )
        nodes.extend(_wrap_with_wait_for_services(controller_cfg, global_namespace, [cm_node]))

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

    for component_name in _component_variant_names(config, "usb_cam"):
        if _component_enabled(config, component_name):
            nodes.extend(_create_usb_cam_nodes(config, global_namespace, component_name=component_name))

    for component_name in _component_variant_names(config, "apriltag_ros"):
        if _component_enabled(config, component_name):
            nodes.extend(_create_apriltag_nodes(config, global_namespace, component_name=component_name))

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

    if _component_enabled(config, "isaac_sim_gpio"):
        nodes.extend(_create_isaac_sim_gpio_nodes(config, global_namespace))

    if _component_enabled(config, "limit_switch_calibration"):
        nodes.extend(_create_limit_switch_calibration_nodes(config, global_namespace))

    if _component_enabled(config, "waypoint_state_machine"):
        nodes.extend(_create_waypoint_state_machine_nodes(config, global_namespace))

    if _component_enabled(config, "i2c_manager"):
        i2c_actions = _create_i2c_manager_nodes(config, global_namespace)
        nodes.extend(
            _wrap_with_wait_for_services(
                _component_config(config, "i2c_manager"),
                global_namespace,
                i2c_actions,
            )
        )

    if _component_enabled(config, "uwb_i2c_reader"):
        uwb_actions = _create_uwb_i2c_reader_nodes(config, global_namespace)
        nodes.extend(
            _wrap_with_wait_for_services(
                _component_config(config, "uwb_i2c_reader"),
                global_namespace,
                uwb_actions,
            )
        )

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

    node_action = Node(
        package=node_cfg.get("package", "gpio_button_event"),
        executable=node_cfg.get("executable", "gpio_button_event_node"),
        name=node_name,
        parameters=parameter_entries,
        remappings=remappings,
        namespace=node_namespace,
        output=node_cfg.get("output", "screen"),
    )

    prime_pullups = bool(node_cfg.get("prime_pullups", False))
    if not prime_pullups:
        return [node_action]

    merged_params: Dict[str, Any] = {}
    for entry in parameter_entries:
        if isinstance(entry, dict):
            merged_params.update(entry)

    gpio_chip, pullup_lines = _extract_gpio_pullups(merged_params)
    if not pullup_lines:
        return [node_action]

    repo_root = _CONFIG_BASE_DIR.parent if _CONFIG_BASE_DIR is not None else Path.cwd()
    prime_script = node_cfg.get("prime_pullups_script")
    if prime_script:
        script_path = _resolve_config_path(prime_script) or prime_script
    else:
        script_path = str(repo_root / "scripts" / "prime_gpio_pullups.sh")

    prime_action = ExecuteProcess(
        cmd=[script_path, gpio_chip, *[str(line) for line in pullup_lines]],
        output=node_cfg.get("output", "screen"),
    )

    return [
        prime_action,
        RegisterEventHandler(OnProcessExit(target_action=prime_action, on_exit=[node_action])),
    ]


def _create_isaac_sim_gpio_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "isaac_sim_gpio")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "isaac_sim_gpio")
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
            package=node_cfg.get("package", "isaac_sim_gpio"),
            executable=node_cfg.get("executable", "isaac_sim_gpio_node"),
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
        resolved_entries = [_resolve_config_path(item) for item in params_entry if item]
        resolved_entries = [item for item in resolved_entries if item]
        return resolved_entries or [default_config]
    if isinstance(params_entry, str) and params_entry.strip():
        resolved = _resolve_config_path(params_entry)
        return [resolved] if resolved else [default_config]
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


def _create_usb_cam_nodes(
    config: Dict[str, Any], global_namespace: str, component_name: str = "usb_cam"
) -> List[Node]:
    node_cfg = _component_config(config, component_name)
    params_file = node_cfg.get("params_file", _discover_default_config("usb_cam.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())
    elif not isinstance(remappings, list):
        remappings = []

    node_name = node_cfg.get("name", "usb_cam")
    parameter_entries: List[Any] = []
    merged_parameters: Dict[str, Any] = {}
    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)
            merged_parameters.update(loaded_parameters)
    if inline_parameters:
        parameter_entries.append(inline_parameters)
        merged_parameters.update(inline_parameters)

    scaled_camera_info_url = _maybe_scale_camera_info(merged_parameters, node_name)
    if scaled_camera_info_url:
        parameter_entries.append({"camera_info_url": scaled_camera_info_url})

    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "usb_cam"),
            executable=node_cfg.get("executable", "usb_cam_node_exe"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_apriltag_nodes(
    config: Dict[str, Any], global_namespace: str, component_name: str = "apriltag_ros"
) -> List[Node]:
    node_cfg = _component_config(config, component_name)
    params_file = node_cfg.get("params_file", _discover_default_config("apriltag_ros.yaml"))
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())
    elif not isinstance(remappings, list):
        remappings = []

    node_name = node_cfg.get("name", "apriltag")
    node_namespace = _resolve_namespace(global_namespace, node_cfg.get("namespace"))
    parameter_entries: List[Any] = []
    if params_file:
        ros_params = _load_ros_parameters_from_file(params_file, node_name)
        ros_params = _prune_empty_apriltag_params(ros_params)
        if ros_params:
            params_path = _write_generated_params_file(node_name, node_namespace, ros_params)
            if params_path:
                parameter_entries.append(params_path)
        if not remappings:
            file_remappings = _load_remappings_from_file(params_file, node_name)
            if file_remappings:
                remappings = file_remappings
    if inline_parameters:
        parameter_entries.append(inline_parameters)

    parameter_entries = _namespace_frame_ids(parameter_entries, global_namespace)

    return [
        Node(
            package=node_cfg.get("package", "apriltag_ros"),
            executable=node_cfg.get("executable", "apriltag_node"),
            name=node_name,
            parameters=parameter_entries,
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


def _create_i2c_manager_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "i2c_manager")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "i2c_manager")
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
            package=node_cfg.get("package", "i2c_manager"),
            executable=node_cfg.get("executable", "i2c_manager_node"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_uwb_i2c_reader_nodes(config: Dict[str, Any], global_namespace: str) -> List[Node]:
    node_cfg = _component_config(config, "uwb_i2c_reader")
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    node_name = node_cfg.get("name", "uwb_i2c_reader")
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
            package=node_cfg.get("package", "uwb_i2c_reader"),
            executable=node_cfg.get("executable", "uwb_i2c_reader_node"),
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
        resolved_params_file = _resolve_config_path(params_file)
        if resolved_params_file:
            base_parameters["config_file"] = resolved_params_file

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
        resolved_path = _resolve_config_path(command_path)
        xacro_path = resolved_path or command_path
        return {
            "robot_description": ParameterValue(
                Command([FindExecutable(name="xacro"), " ", xacro_path]),
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
    """Prefer config/filename in CWD, else search upward from this file."""
    cwd_candidate = Path.cwd() / "config" / filename
    if cwd_candidate.exists():
        return str(cwd_candidate)
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "config" / filename
        if candidate.exists():
            return str(candidate)
    # Fallback to a relative config/filename inside the current working directory.
    return str(cwd_candidate)


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


def _wrap_with_wait_for_services(
    component_cfg: Dict[str, Any],
    global_namespace: str,
    actions: List[Any],
) -> List[Any]:
    wait_for = component_cfg.get("wait_for_services")
    if not wait_for:
        return actions
    if isinstance(wait_for, str):
        services = [wait_for]
    elif isinstance(wait_for, list):
        services = wait_for
    else:
        return actions

    wait_services = [
        _namespaced_topic(global_namespace, service)
        for service in services
        if isinstance(service, str) and service.strip()
    ]
    if not wait_services:
        return actions

    output = component_cfg.get("output", "screen")
    wait_actions: List[Any] = []
    previous_action: ExecuteProcess | None = None
    for service in wait_services:
        wait_cmd = f"until ros2 service list | grep -q '^{service}$'; do sleep 0.2; done"
        wait_action = ExecuteProcess(cmd=["bash", "-lc", wait_cmd], output=output)
        if previous_action is None:
            wait_actions.append(wait_action)
        else:
            wait_actions.append(
                RegisterEventHandler(
                    OnProcessExit(target_action=previous_action, on_exit=[wait_action])
                )
            )
        previous_action = wait_action

    wait_actions.append(
        RegisterEventHandler(
            OnProcessExit(target_action=previous_action, on_exit=actions)
        )
    )
    return wait_actions


def _load_topics(config_path: str) -> Dict[str, Any]:
    resolved_path = _resolve_config_path(config_path)
    if not resolved_path:
        return {}
    path = Path(resolved_path)
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


def _load_ros_parameters_from_file(path_str: str, node_name: str) -> Dict[str, Any]:
    resolved_path = _resolve_config_path(path_str)
    if not resolved_path:
        return {}
    path = Path(resolved_path)
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
    return {}


def _prune_empty_apriltag_params(params: Dict[str, Any]) -> Dict[str, Any]:
    """Remove empty tag filters so apriltag_ros doesn't error on empty arrays."""
    if not isinstance(params, dict):
        return params
    tag_cfg = params.get("tag")
    if not isinstance(tag_cfg, dict):
        return params
    ids = tag_cfg.get("ids")
    if isinstance(ids, list) and not ids:
        params = dict(params)
        params.pop("tag", None)
        return params
    tag_cfg = dict(tag_cfg)
    for key in ("frames", "sizes"):
        value = tag_cfg.get(key)
        if isinstance(value, list) and not value:
            tag_cfg.pop(key, None)
    params = dict(params)
    params["tag"] = tag_cfg
    return params


def _load_remappings_from_file(path_str: str, node_name: str) -> List[Any]:
    resolved_path = _resolve_config_path(path_str)
    if not resolved_path:
        return []
    path = Path(resolved_path)
    data = _load_yaml_file(path)
    if not data:
        return []
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
    remappings = entry.get("remappings") if isinstance(entry, dict) else None
    if remappings is None:
        remappings = data.get("remappings") if isinstance(data, dict) else None
    if isinstance(remappings, dict):
        return list(remappings.items())
    if isinstance(remappings, list):
        return remappings
    return []


def _write_generated_params_file(node_name: str, node_namespace: str, params: Dict[str, Any]) -> str | None:
    if not params:
        return None
    ros_home = Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros")))
    target_dir = ros_home / "robot_main" / "params"
    target_dir.mkdir(parents=True, exist_ok=True)
    safe_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in node_name)
    target_path = target_dir / f"{safe_name}_params.yaml"
    if node_namespace:
        full_name = f"{node_namespace.rstrip('/')}/{node_name}"
    else:
        full_name = node_name
    payload = {full_name: {"ros__parameters": params}}
    with open(target_path, "w", encoding="utf-8") as config_file:
        yaml.safe_dump(payload, config_file, sort_keys=False)
    return str(target_path)


def _maybe_scale_camera_info(parameters: Dict[str, Any], node_name: str) -> str | None:
    camera_info_url = parameters.get("camera_info_url")
    if not isinstance(camera_info_url, str) or not camera_info_url.strip():
        return None
    width = _coerce_int(parameters.get("image_width"))
    height = _coerce_int(parameters.get("image_height"))
    if not width or not height:
        return None

    base_url = (
        parameters.get("camera_info_calibration_url")
        or parameters.get("camera_info_base_url")
        or camera_info_url
    )
    if not isinstance(base_url, str) or not base_url.strip():
        return None

    base_path = _resolve_camera_info_url(base_url)
    if not base_path:
        return None
    base_info = _load_yaml_file(Path(base_path))
    if not base_info:
        return None

    scaled_info = _scale_camera_info(base_info, width, height, parameters.get("camera_name") or node_name)
    if not scaled_info:
        return None

    output_path = _resolve_camera_info_output_path(camera_info_url)
    if not output_path:
        output_path = _default_camera_info_path(node_name, width, height)

    existing_info = _load_yaml_file(Path(output_path)) if Path(output_path).exists() else {}
    if existing_info and _camera_info_equivalent(existing_info, scaled_info):
        return _format_camera_info_url(output_path)

    scaled_path = _write_camera_info_file(scaled_info, node_name, width, height, output_path)
    return _format_camera_info_url(scaled_path) if scaled_path else None


def _coerce_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _resolve_camera_info_url(url: str) -> str | None:
    if not url:
        return None
    if url.startswith("package://"):
        remainder = url[len("package://") :]
        if "/" not in remainder:
            return None
        pkg_name, rel_path = remainder.split("/", 1)
        try:
            pkg_share = get_package_share_directory(pkg_name)
        except (LookupError, ValueError):
            return None
        return str(Path(pkg_share) / rel_path)
    if url.startswith("file://"):
        return url[len("file://") :]
    return _resolve_config_path(url)


def _format_camera_info_url(path: str) -> str:
    return f"file://{path}"


def _scale_camera_info(
    base_info: Dict[str, Any], width: int, height: int, camera_name: str | None
) -> Dict[str, Any] | None:
    base_width = _coerce_int(base_info.get("image_width"))
    base_height = _coerce_int(base_info.get("image_height"))
    if not base_width or not base_height:
        return None
    if base_width <= 0 or base_height <= 0:
        return None

    scale_x = float(width) / float(base_width)
    scale_y = float(height) / float(base_height)

    scaled = deepcopy(base_info)
    scaled["image_width"] = int(width)
    scaled["image_height"] = int(height)
    if camera_name:
        scaled["camera_name"] = camera_name

    camera_matrix = scaled.get("camera_matrix")
    if isinstance(camera_matrix, dict):
        scaled["camera_matrix"] = _scale_camera_matrix(camera_matrix, scale_x, scale_y)

    projection_matrix = scaled.get("projection_matrix")
    if isinstance(projection_matrix, dict):
        scaled["projection_matrix"] = _scale_projection_matrix(projection_matrix, scale_x, scale_y)

    return scaled


def _scale_camera_matrix(matrix_entry: Dict[str, Any], scale_x: float, scale_y: float) -> Dict[str, Any]:
    data = matrix_entry.get("data")
    if not isinstance(data, list) or len(data) < 9:
        return matrix_entry
    scaled_data = list(data)
    scaled_data[0] = scaled_data[0] * scale_x
    scaled_data[2] = scaled_data[2] * scale_x
    scaled_data[4] = scaled_data[4] * scale_y
    scaled_data[5] = scaled_data[5] * scale_y
    updated = dict(matrix_entry)
    updated["data"] = scaled_data
    return updated


def _scale_projection_matrix(matrix_entry: Dict[str, Any], scale_x: float, scale_y: float) -> Dict[str, Any]:
    data = matrix_entry.get("data")
    if not isinstance(data, list) or len(data) < 12:
        return matrix_entry
    scaled_data = list(data)
    scaled_data[0] = scaled_data[0] * scale_x
    scaled_data[2] = scaled_data[2] * scale_x
    scaled_data[3] = scaled_data[3] * scale_x
    scaled_data[5] = scaled_data[5] * scale_y
    scaled_data[6] = scaled_data[6] * scale_y
    updated = dict(matrix_entry)
    updated["data"] = scaled_data
    return updated


def _camera_info_equivalent(left: Any, right: Any, atol: float = 1e-9) -> bool:
    if isinstance(left, dict) and isinstance(right, dict):
        if left.keys() != right.keys():
            return False
        return all(_camera_info_equivalent(left[key], right[key], atol) for key in left)
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return False
        return all(_camera_info_equivalent(lv, rv, atol) for lv, rv in zip(left, right))
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right)) <= atol
    return left == right


def _default_camera_info_path(node_name: str, width: int, height: int) -> str:
    ros_home = Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros")))
    target_dir = ros_home / "camera_info"
    target_dir.mkdir(parents=True, exist_ok=True)
    safe_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in node_name)
    target_path = target_dir / f"{safe_name}_{width}x{height}.yaml"
    return str(target_path)


def _resolve_camera_info_output_path(url: str) -> str | None:
    if not url:
        return None
    if url.startswith("package://"):
        return None
    if url.startswith("file://"):
        return url[len("file://") :]
    resolved = _resolve_config_path(url)
    return resolved


def _write_camera_info_file(
    camera_info: Dict[str, Any], node_name: str, width: int, height: int, output_path: str
) -> str | None:
    target_path = Path(output_path)
    if not target_path.parent.exists():
        target_path.parent.mkdir(parents=True, exist_ok=True)
    with open(target_path, "w", encoding="utf-8") as config_file:
        yaml.safe_dump(camera_info, config_file, sort_keys=False)
    return str(target_path)


def _resolve_config_path(path_str: str | None) -> str | None:
    if not path_str:
        return None
    expanded = os.path.expandvars(os.path.expanduser(path_str))
    path = Path(expanded)
    if path.is_absolute() or _CONFIG_BASE_DIR is None:
        return str(path)
    return str(_CONFIG_BASE_DIR / path)


def _load_parameters_from_file(path_str: str, node_name: str) -> Dict[str, Any]:
    resolved_path = _resolve_config_path(path_str)
    if not resolved_path:
        return {}
    path = Path(resolved_path)
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


def _extract_gpio_pullups(parameters: Dict[str, Any]) -> tuple[str, List[int]]:
    gpio_chip = parameters.get("gpio_chip", "gpiochip0")
    switches = parameters.get("switches", [])
    pullup_lines: List[int] = []
    default_pullup = bool(parameters.get("use_internal_pullup", False))
    default_line = parameters.get("gpio_line", 4)

    if isinstance(switches, list) and switches:
        for name in switches:
            pullup = parameters.get(f"switches.{name}.pullup", default_pullup)
            if not pullup:
                continue
            line = parameters.get(f"switches.{name}.gpio_line", default_line)
            if isinstance(line, int):
                pullup_lines.append(line)
    else:
        if default_pullup and isinstance(default_line, int):
            pullup_lines.append(default_line)

    unique_lines = sorted(set(pullup_lines))
    return str(gpio_chip), unique_lines


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
            elif _is_service_key(key):
                updated[key] = _namespaced_topic(global_namespace, value)
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


def _is_service_key(key: Any) -> bool:
    if not isinstance(key, str):
        return False
    lowered = key.lower()
    return lowered.endswith("_service") or lowered.endswith("_service_name") or lowered == "service_name"


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
