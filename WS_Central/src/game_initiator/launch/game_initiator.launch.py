import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=_discover_default_params(),
        description='Absolute path to game_initiator YAML parameters file.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    node = Node(
        package='game_initiator',
        executable='game_initiator_node',
        name='game_initiator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([params_file_arg, use_sim_time_arg, node])


def _discover_default_params() -> str:
    candidates: list[Path] = []

    ws_central = os.getenv('WS_CENTRAL', '')
    if ws_central:
        candidates.append(
            Path(os.path.expandvars(os.path.expanduser(ws_central))) / 'config' / 'game_initiator.yaml'
        )

    candidates.append(Path.cwd() / 'config' / 'game_initiator.yaml')

    try:
        share_dir = Path(get_package_share_directory('game_initiator'))
        candidates.append(share_dir / 'config' / 'game_initiator.yaml')
    except Exception:
        pass

    seen: set[str] = set()
    for candidate in candidates:
        resolved = candidate.expanduser()
        key = str(resolved)
        if key in seen:
            continue
        seen.add(key)
        if resolved.exists():
            return str(resolved)

    return str(candidates[0] if candidates else Path.cwd() / 'config' / 'game_initiator.yaml')
