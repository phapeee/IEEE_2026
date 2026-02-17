import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    central_share = Path(get_package_share_directory('central_main'))
    central_config_dir = central_share / "config"

    ground_config = _discover_config('ground_fleet.yaml', central_config_dir)
    ground_graph = _discover_config('ground_nav_graph.yaml', central_config_dir)
    drone_config = _discover_config('drone_fleet.yaml', central_config_dir)
    drone_graph = _discover_config('drone_nav_graph.yaml', central_config_dir)

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time for both fleet adapters',
        ),
        Node(
            package='ieee_fleet_adapter',
            executable='fleet_adapter',
            name='ground_fleet_adapter',
            output='screen',
            arguments=['-c', ground_config, '-n', ground_graph],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='ieee_fleet_adapter',
            executable='fleet_adapter',
            name='drone_fleet_adapter',
            output='screen',
            arguments=['-c', drone_config, '-n', drone_graph],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])


def _discover_config(filename: str, central_config_dir: Path | None = None) -> str:
    """Prefer WS_CENTRAL/config, then central_main config, then config/filename in CWD, else search upward."""
    ws_central = os.getenv('WS_CENTRAL', '')
    if ws_central:
        ws_candidate = Path(os.path.expandvars(os.path.expanduser(ws_central))) / 'config' / filename
        if ws_candidate.exists():
            return str(ws_candidate)

    if central_config_dir is not None:
        central_candidate = central_config_dir / filename
        if central_candidate.exists():
            return str(central_candidate)
    cwd_candidate = Path.cwd() / "config" / filename
    if cwd_candidate.exists():
        return str(cwd_candidate)
    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "config" / filename
        if candidate.exists():
            return str(candidate)
    return str(cwd_candidate)
