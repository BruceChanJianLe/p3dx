#!/usr/bin/python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    this_package_path = get_package_share_directory("p3dx_control")
    ekf_params_file = LaunchConfiguration(
        "ekf_params_file",
        default=join(this_package_path, "config", "localization.yaml"),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument("ekf_params_file", default_value=ekf_params_file),
            ekf_node,
        ]
    )
