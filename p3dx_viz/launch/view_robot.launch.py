#!/usr/bin/python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare("p3dx_viz"), "rviz2", "pioneer_a_nav.rviz"]
  )

  remappings = [
          ('/navigate_to_pose', '/pioneer_a/navigate_to_pose'),
          ('/navigate_through_poses', '/pioneer_a/navigate_through_poses'),
          ]

  node_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    arguments=['-d', rviz_config_file],
    remappings=remappings,
    namespace='pioneer_a',

    output='screen'
  )

  return LaunchDescription([
      node_rviz
  ])
