#!/usr/bin/python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare("p3dx_viz"), "rviz2", "nav2_view.rviz"]
  )

  node_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    arguments=['-d', rviz_config_file],

    output='screen'
  )

  return LaunchDescription([
      node_rviz
  ])
