#!/usr/bin/python3

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_package_path = get_package_share_directory("p3dx_description")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    lidar_enabled = LaunchConfiguration("lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        join(this_package_path, "urdf/p3dx/pioneer3dx.xacro"),
                        " camera_enabled:=",
                        camera_enabled,
                        " lidar_enabled:=",
                        lidar_enabled,
                        " odometry_source:=",
                        odometry_source,
                    ]
                )
            }
        ],
        remappings=[
            ("/joint_states", "pioneer/joint_states"),
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        remappings=[
            ("/joint_states", "pioneer/joint_states"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
            DeclareLaunchArgument("lidar_enabled", default_value=lidar_enabled),
            DeclareLaunchArgument("odometry_source", default_value="encoders"),
            robot_state_publisher,
            joint_state_publisher,
        ]
    )
