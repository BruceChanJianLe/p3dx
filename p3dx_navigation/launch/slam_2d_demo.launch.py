#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  this_package_path = get_package_share_directory("p3dx_navigation")
  robot_namespace = LaunchConfiguration("robot_namespace", default="")

  slam_2d_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(join(this_package_path, "launch", "slam_2d.launch.py")),
    launch_arguments={
      "robot_namespace" : robot_namespace,
    }.items()
  )

  navigation_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(join(this_package_path, "launch", "navigation.launch.py")),
    launch_arguments={
      "robot_namespace" : robot_namespace,
    }.items()
  )

  return LaunchDescription([
    DeclareLaunchArgument("robot_namespace", default_value=""),
    slam_2d_launch,
    navigation_launch,
  ])
