import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description() -> LaunchDescription:
    default_pool = _discover_default_config('task_pool.yaml')
    default_params = _discover_default_config('game_director.yaml')

    task_pool_arg = DeclareLaunchArgument(
        'task_pool_path',
        default_value=default_pool,
        description='Absolute path to task_pool.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Absolute path to game_director.yaml',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    node = Node(
        package='game_director',
        executable='game_director',
        name='game_director',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'task_pool_path': LaunchConfiguration('task_pool_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    return LaunchDescription([task_pool_arg, params_file_arg, use_sim_time_arg, node])



def _discover_default_config(filename: str) -> str:
    candidates: list[Path] = []

    ws_central = os.getenv('WS_CENTRAL', '')
    if ws_central:
        candidates.append(Path(os.path.expandvars(os.path.expanduser(ws_central))) / 'config' / filename)

    candidates.append(Path.cwd() / 'config' / filename)

    try:
        share = Path(get_package_share_directory('game_director'))
        candidates.append(share / 'config' / filename)
    except Exception:
        pass

    # Works both from source and from install space:
    # .../src/game_director/launch -> .../WS_Central/config
    # .../install/share/game_director/launch -> .../WS_Central/config
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        candidates.append(parent / 'config' / filename)

    seen: set[str] = set()
    for candidate in candidates:
        resolved = Path(os.path.expandvars(str(candidate.expanduser())))
        key = str(resolved)
        if key in seen:
            continue
        seen.add(key)
        if resolved.exists():
            return str(resolved)

    return str(candidates[0])
