#!/usr/bin/python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    this_package_path = get_package_share_directory("p3dx_navigation")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "localization.launch.py")
        ),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "navigation.launch.py")
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_name",
                default_value="warehouse_world",
                description="Use to select 2d map name",
            ),
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="",
                description="Add namespace for nodes related to this robot",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "initial_pose_x",
                default_value="0.0",
                description="AMCL initial pose x",
            ),
            DeclareLaunchArgument(
                "initial_pose_y",
                default_value="0.0",
                description="AMCL initial pose y",
            ),
            DeclareLaunchArgument(
                "initial_pose_z",
                default_value="0.0",
                description="AMCL initial pose z",
            ),
            DeclareLaunchArgument(
                "initial_pose_yaw",
                default_value="0.0",
                description="AMCL initial pose yaw",
            ),
            localization_launch,
            navigation_launch,
        ]
    )
