#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    this_package_path = get_package_share_directory("p3dx_viz")
    robot_namespace = LaunchConfiguration("robot_namespace", default="")
    params_file = LaunchConfiguration("params_file")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("p3dx_viz"), "rviz2", "nav2_view.rviz"]
    )

    remappings = [
        # ('/navigate_to_pose', '/namespace/navigate_to_pose'),
        # ('/navigate_through_poses', '/namespace/navigate_through_poses'),
    ]

    params_file = ReplaceString(
        source_file=params_file,
        replacements={
            "<tf_prefix>": (robot_namespace, "/"),
            "<tf_prefix_name>": (robot_namespace),
            "<namespace>": ("/", robot_namespace),
        },
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={
            "<tf_prefix>": (robot_namespace),
            "<tf_prefix_name>": (robot_namespace),
            "<namespace>": (robot_namespace),
        },
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
    )

    configured_params = (
        RewrittenYaml(
            source_file=params_file, root_key="", param_rewrites={}, convert_types=True
        ),
    )

    node_rviz_namespace = Node(
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
        namespace=robot_namespace,
        package="rviz2",
        executable="rviz2",
        name="p3dx_rviz",
        arguments=["-d", configured_params],
        remappings=remappings,
        output="screen",
    )

    node_rviz = Node(
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
        package="rviz2",
        executable="rviz2",
        name="p3dx_rviz",
        arguments=["-d", configured_params],
        remappings=remappings,
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    this_package_path, "rviz2", "nav2_view.rviz"
                ),
                description="Full path to rviz2 config file",
            ),
            node_rviz,
            node_rviz_namespace,
        ]
    )
