#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  this_package_path = get_package_share_directory("p3dx_navigation")

  localization_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(join(this_package_path, "launch", "localization.launch.py")),
    launch_arguments={
      "map_name" : "warehouse_world",
      "robot_namespace" : "pioneer_a",
    }.items()
  )

  navigation_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(join(this_package_path, "launch", "navigation.launch.py")),
    launch_arguments={
      "robot_namespace" : "pioneer_a",
    }.items()
  )

  return LaunchDescription([
    localization_launch,
    navigation_launch,
  ])
