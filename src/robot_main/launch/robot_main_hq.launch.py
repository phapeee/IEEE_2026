"""HQ-only launch: bring up nav2 map server + lifecycle manager without global namespace."""

from pathlib import Path
from typing import Any, Dict, List

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config_path = _discover_hq_config("robot_main.yaml")
    config_arg = DeclareLaunchArgument(
        "robot_main_config",
        default_value=default_config_path,
        description="Absolute path to the HQ YAML configuration file.",
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=_launch_components),
    ])


def _launch_components(context, *_: Any) -> List[Node]:
    config_path = Path(context.perform_substitution(LaunchConfiguration("robot_main_config")))
    config = _load_robot_main_config(config_path)

    nodes: List[Node] = []

    if _component_enabled(config, "nav2_map_server"):
        nodes.extend(_create_nav2_map_server_nodes(config))

    if _component_enabled(config, "nav2_lifecycle_manager"):
        nodes.extend(_create_nav2_lifecycle_manager_nodes(config))

    return nodes


def _create_nav2_map_server_nodes(config: Dict[str, Any]) -> List[Node]:
    node_cfg = _component_config(config, "nav2_map_server")
    params_file = node_cfg.get("params_file", _discover_hq_config("nav2_map_server.yaml"))
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

    node_namespace = _normalize_namespace(node_cfg.get("namespace"))

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


def _create_nav2_lifecycle_manager_nodes(config: Dict[str, Any]) -> List[Node]:
    node_cfg = _component_config(config, "nav2_lifecycle_manager")
    params_file = node_cfg.get("params_file", _discover_hq_config("lifecycle_localization.yaml"))
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

    node_namespace = _normalize_namespace(node_cfg.get("namespace"))

    return [
        Node(
            package=node_cfg.get("package", "nav2_lifecycle_manager"),
            executable=node_cfg.get("executable", "lifecycle_manager"),
            name=node_name,
            parameters=parameter_entries,
            remappings=remappings,
            namespace=node_namespace,
            output=node_cfg.get("output", "screen"),
        )
    ]


def _discover_hq_config(filename: str) -> str:
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "config_HQ" / filename
        if candidate.exists():
            return str(candidate)
    return str(Path.cwd() / "config_HQ" / filename)


def _component_config(config: Dict[str, Any], name: str) -> Dict[str, Any]:
    components = config.get("components", {})
    entry = components.get(name, {})
    if isinstance(entry, bool):
        return {"enabled": entry}
    return entry or {}


def _component_enabled(config: Dict[str, Any], name: str) -> bool:
    entry = _component_config(config, name)
    return entry.get("enabled", False)


def _load_robot_main_config(config_path: Path) -> Dict[str, Any]:
    if not config_path.exists():
        return {}
    with open(config_path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    return data


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
