#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    robot_namespace = LaunchConfiguration("robot_namespace", default="")

    this_package_path = get_package_share_directory("p3dx_control")
    ekf_params_file = LaunchConfiguration(
        "ekf_params_file",
        default=join(this_package_path, "config", "localization.yaml"),
    )

    ekf_params_file = ReplaceString(
        source_file=ekf_params_file,
        replacements={"<tf_prefix>": (robot_namespace, "/")},
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
    )

    ekf_params_file = ReplaceString(
        source_file=ekf_params_file,
        replacements={"<tf_prefix>": (robot_namespace)},
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("robot_namespace"), "")
        ),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=ekf_params_file,
            root_key=robot_namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Fuses RosAria/odom (wheel encoders) + imu/data and publishes
    # odometry/filtered as well as the odom->base_link transform.
    # Spawn the robot with odometry_source:=ekf so the gazebo diff-drive
    # plugin does not publish a competing odom->base_link TF.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=join(this_package_path, "config", "localization.yaml"),
            ),
            DeclareLaunchArgument(
                "robot_namespace", default_value="", description="Top-level namespace"
            ),
            ekf_node,
        ]
    )
