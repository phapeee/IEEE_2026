from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    declare_config = DeclareLaunchArgument(
        "config_file",
        default_value="/ws/config/uwb_config.yaml",
        description="Absolute path to the uwb_triangulaion configuration file.",
    )

    triangulation_node = Node(
        package="uwb_triangulaion",
        executable="uwb_triangulaion_node",
        name="uwb_triangulaion_node",
        parameters=[{"config_file": config_file}],
    )

    covariance_node = Node(
        package="uwb_triangulaion",
        executable="uwb_covariance_estimator_node",
        name="uwb_covariance_estimator_node",
        parameters=[{"config_file": config_file}],
    )

    return LaunchDescription([
        declare_config,
        triangulation_node,
        covariance_node,
    ])

