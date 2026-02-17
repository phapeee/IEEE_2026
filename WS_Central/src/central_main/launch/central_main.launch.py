"""Launch description for the central hub RMF stack + adapter launcher."""

import os
from pathlib import Path
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_CONFIG_BASE_DIR: Path | None = None


def generate_launch_description() -> LaunchDescription:
    default_config_path = _discover_default_config("central_main.yaml")
    config_arg = DeclareLaunchArgument(
        "central_main_config",
        default_value=default_config_path,
        description="Absolute path to the YAML configuration file that holds all robot parameters.",
    )

    return LaunchDescription(
        [
            config_arg,
            OpaqueFunction(function=_launch_components),
        ]
    )


def _launch_components(context, *_: Any) -> List[Any]:
    global _CONFIG_BASE_DIR
    config_value = context.perform_substitution(LaunchConfiguration("central_main_config"))
    config_path = Path(os.path.expandvars(os.path.expanduser(config_value)))
    if not config_path.is_absolute():
        config_path = Path.cwd() / config_path
    _CONFIG_BASE_DIR = config_path.parent

    config = _load_central_main_config(config_path)
    global_namespace = _normalize_namespace(config.get("namespace", ""))

    nodes: List[Any] = []
    if _component_enabled(config, "rmf_stack"):
        nodes.extend(_create_rmf_stack_nodes(config))
    if _component_enabled(config, "game_initiator"):
        nodes.extend(_create_game_initiator_nodes(config))
    if _component_enabled(config, "game_director"):
        nodes.extend(_create_game_director_nodes(config))
    return nodes


def _create_rmf_stack_nodes(config: Dict[str, Any]) -> List[Any]:
    rmf_cfg = _component_config(config, "rmf_stack")
    if not rmf_cfg:
        return []

    use_sim_time = bool(rmf_cfg.get("use_sim_time", False))
    output = rmf_cfg.get("output", "screen")
    startup_delay_sec = float(rmf_cfg.get("startup_delay_sec", 2.0))

    schedule_cfg = _subcomponent_config(rmf_cfg, "schedule")
    dispatcher_cfg = _subcomponent_config(rmf_cfg, "dispatcher")
    fleet_cfg = _subcomponent_config(rmf_cfg, "fleet_adapters")

    nodes: List[Any] = []
    if _subcomponent_enabled(schedule_cfg):
        nodes.append(
            _create_simple_node(
                schedule_cfg,
                default_package="rmf_traffic_ros2",
                default_executable="rmf_traffic_schedule",
                default_name="rmf_traffic_schedule",
                use_sim_time=use_sim_time,
                output=output,
            )
        )

    if _subcomponent_enabled(dispatcher_cfg):
        nodes.append(
            _create_simple_node(
                dispatcher_cfg,
                default_package="rmf_task_ros2",
                default_executable="rmf_task_dispatcher",
                default_name="rmf_task_dispatcher",
                use_sim_time=use_sim_time,
                output=output,
            )
        )

    adapter_nodes = _create_fleet_adapter_nodes(
        fleet_cfg,
        use_sim_time=use_sim_time,
        output=output,
    )
    if adapter_nodes:
        if startup_delay_sec > 0.0:
            nodes.append(
                TimerAction(
                    period=startup_delay_sec,
                    actions=adapter_nodes,
                )
            )
        else:
            nodes.extend(adapter_nodes)

    return nodes


def _create_fleet_adapter_nodes(
    fleet_cfg: Dict[str, Any],
    use_sim_time: bool,
    output: str,
) -> List[Any]:
    if not _subcomponent_enabled(fleet_cfg):
        return []

    central_share = Path(get_package_share_directory("central_main"))
    central_config_dir = central_share / "config"

    repo_ground = _resolve_config_path("ground_fleet.yaml")
    repo_drone = _resolve_config_path("drone_fleet.yaml")
    repo_ground_graph = _resolve_config_path("ground_nav_graph.yaml")
    repo_drone_graph = _resolve_config_path("drone_nav_graph.yaml")

    if repo_ground and not Path(repo_ground).exists():
        repo_ground = None
    if repo_drone and not Path(repo_drone).exists():
        repo_drone = None
    if repo_ground_graph and not Path(repo_ground_graph).exists():
        repo_ground_graph = None
    if repo_drone_graph and not Path(repo_drone_graph).exists():
        repo_drone_graph = None
    defaults = {
        "ground": {
            "config": repo_ground or str(central_config_dir / "ground_fleet.yaml"),
            "nav_graph": repo_ground_graph or str(central_config_dir / "ground_nav_graph.yaml"),
            "name": "ground_fleet_adapter",
        },
        "drone": {
            "config": repo_drone or str(central_config_dir / "drone_fleet.yaml"),
            "nav_graph": repo_drone_graph or str(central_config_dir / "drone_nav_graph.yaml"),
            "name": "drone_fleet_adapter",
        },
    }

    server_uri = fleet_cfg.get("server_uri", "")
    fleet_use_sim_time = bool(fleet_cfg.get("use_sim_time", use_sim_time))

    nodes: List[Any] = []
    for fleet_name, default_cfg in defaults.items():
        entry = _subcomponent_config(fleet_cfg, fleet_name)
        if not _subcomponent_enabled(entry):
            continue

        config_path = _resolve_config_path(entry.get("config")) or default_cfg["config"]
        nav_graph_path = _resolve_config_path(entry.get("nav_graph")) or default_cfg["nav_graph"]
        adapter_name = entry.get("name", default_cfg["name"])

        arguments: List[str] = ["-c", config_path, "-n", nav_graph_path]
        if fleet_use_sim_time or bool(entry.get("use_sim_time", False)):
            arguments.append("--use_sim_time")

        base_args = fleet_cfg.get("arguments", [])
        if isinstance(base_args, (list, tuple)):
            arguments.extend([str(arg) for arg in base_args])

        entry_args = entry.get("arguments", [])
        if isinstance(entry_args, (list, tuple)):
            arguments.extend([str(arg) for arg in entry_args])

        parameters: List[Any] = []
        parameters.extend(_build_parameters(fleet_cfg, adapter_name, use_sim_time=None))
        parameters.extend(_build_parameters(entry, adapter_name, use_sim_time=None))

        extra_params: Dict[str, Any] = {}
        if server_uri:
            extra_params["server_uri"] = server_uri
        if fleet_use_sim_time or bool(entry.get("use_sim_time", False)):
            extra_params["use_sim_time"] = True
        if extra_params:
            parameters.append(extra_params)

        node_output = entry.get("output", fleet_cfg.get("output", output))
        node_namespace = _normalize_namespace(entry.get("namespace", ""))

        nodes.append(
            Node(
                package="ieee_fleet_adapter",
                executable="fleet_adapter",
                name=adapter_name,
                output=node_output,
                arguments=arguments,
                parameters=parameters,
                namespace=node_namespace,
            )
        )

    return nodes


def _create_simple_node(
    node_cfg: Dict[str, Any],
    default_package: str,
    default_executable: str,
    default_name: str,
    use_sim_time: bool,
    output: str,
) -> Node:
    node_name = node_cfg.get("name", default_name)
    parameters = _build_parameters(node_cfg, node_name, use_sim_time=use_sim_time)
    remappings = node_cfg.get("remappings", [])
    if isinstance(remappings, dict):
        remappings = list(remappings.items())
    arguments = node_cfg.get("arguments", [])
    if not isinstance(arguments, (list, tuple)):
        arguments = []

    node_namespace = _normalize_namespace(node_cfg.get("namespace", ""))

    return Node(
        package=node_cfg.get("package", default_package),
        executable=node_cfg.get("executable", default_executable),
        name=node_name,
        output=node_cfg.get("output", output),
        parameters=parameters,
        remappings=remappings,
        arguments=list(arguments),
        namespace=node_namespace,
    )


def _create_game_initiator_nodes(config: Dict[str, Any]) -> List[Node]:
    node_cfg = _component_config(config, "game_initiator")
    if not node_cfg:
        return []

    return [
        _create_simple_node(
            node_cfg,
            default_package="game_initiator",
            default_executable="game_initiator_node",
            default_name="game_initiator",
            use_sim_time=bool(node_cfg.get("use_sim_time", False)),
            output=node_cfg.get("output", "screen"),
        )
    ]


def _create_game_director_nodes(config: Dict[str, Any]) -> List[Node]:
    node_cfg = _component_config(config, "game_director")
    if not node_cfg:
        return []

    resolved_cfg = dict(node_cfg)
    if not resolved_cfg.get("params_file"):
        default_params = _resolve_config_path("game_director.yaml") or _discover_default_config("game_director.yaml")
        resolved_cfg["params_file"] = default_params

    inline_parameters = resolved_cfg.get("ros__parameters", {})
    if isinstance(inline_parameters, dict):
        inline_parameters = dict(inline_parameters)
    else:
        inline_parameters = {}

    task_pool_path = inline_parameters.get("task_pool_path")
    if task_pool_path:
        resolved_task_pool = _resolve_config_path(str(task_pool_path))
    else:
        resolved_task_pool = _resolve_config_path("task_pool.yaml") or _discover_default_config("task_pool.yaml")
    if resolved_task_pool:
        inline_parameters["task_pool_path"] = resolved_task_pool

    resolved_cfg["ros__parameters"] = inline_parameters

    return [
        _create_simple_node(
            resolved_cfg,
            default_package="game_director",
            default_executable="game_director",
            default_name="game_director",
            use_sim_time=bool(resolved_cfg.get("use_sim_time", False)),
            output=resolved_cfg.get("output", "screen"),
        )
    ]


def _build_parameters(
    node_cfg: Dict[str, Any],
    node_name: str,
    use_sim_time: bool | None,
) -> List[Any]:
    parameter_entries: List[Any] = []
    params_file = node_cfg.get("params_file")
    inline_parameters = node_cfg.get("ros__parameters", {})

    if params_file:
        loaded_parameters = _load_parameters_from_file(params_file, node_name)
        if loaded_parameters:
            parameter_entries.append(loaded_parameters)

    if inline_parameters:
        parameter_entries.append(inline_parameters)

    if use_sim_time:
        parameter_entries.append({"use_sim_time": True})

    return parameter_entries


def _discover_default_config(filename: str) -> str:
    """Prefer WS_CENTRAL/config first, then config/filename in CWD, else search upward from this file."""
    ws_central = os.getenv("WS_CENTRAL", "")
    if ws_central:
        ws_candidate = Path(os.path.expandvars(os.path.expanduser(ws_central))) / "config" / filename
        if ws_candidate.exists():
            return str(ws_candidate)

    cwd_candidate = Path.cwd() / "config" / filename
    if cwd_candidate.exists():
        return str(cwd_candidate)
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "config" / filename
        if candidate.exists():
            return str(candidate)
    return str(cwd_candidate)


def _load_central_main_config(config_path: Path) -> Dict[str, Any]:
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
    if not entry:
        return False
    if "enabled" not in entry:
        return True
    return bool(entry.get("enabled"))


def _subcomponent_config(parent: Dict[str, Any], name: str) -> Dict[str, Any]:
    entry = parent.get(name, {})
    if isinstance(entry, bool):
        return {"enabled": entry}
    return entry or {}


def _subcomponent_enabled(entry: Dict[str, Any]) -> bool:
    if not entry:
        return False
    if "enabled" not in entry:
        return True
    return bool(entry.get("enabled"))


def _resolve_config_path(path_str: str | None) -> str | None:
    if not path_str:
        return None
    expanded = os.path.expandvars(os.path.expanduser(path_str))
    path = Path(expanded)
    if path.is_absolute() or _CONFIG_BASE_DIR is None:
        return str(path)
    return str(_CONFIG_BASE_DIR / path)


def _load_yaml_file(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    return data if isinstance(data, dict) else {}


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
