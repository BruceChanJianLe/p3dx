#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace", default="")

    # Interactive marker to drive robot in RViz
    twist_marker_server_namespace = Node(
        package="interactive_marker_twist_server",
        executable="marker_server",
        name="twist_marker_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "link_name": [robot_namespace, TextSubstitution(text="/base_link")],
                "robot_name": robot_namespace,
            }
        ],
        remappings=[
            ("cmd_vel", "RosAria/cmd_vel"),
        ],
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
    )

    twist_marker_server = Node(
        package="interactive_marker_twist_server",
        executable="marker_server",
        name="twist_marker_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "link_name": "base_link",
                "robot_name": "",
            }
        ],
        remappings=[
            ("cmd_vel", "RosAria/cmd_vel"),
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace", default_value="", description="Top-level namespace"
            ),
            twist_marker_server_namespace,
            twist_marker_server,
        ]
    )
