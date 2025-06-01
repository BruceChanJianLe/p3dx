import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    lidar_enabled = LaunchConfiguration("lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("p3dx_viz"), "rviz2", "model.rviz"]
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    launch_p3dx_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("p3dx_description"),
                "launch/description.launch.py",
            )
        ),
        launch_arguments={
            "camera_enabled": camera_enabled,
            "lidar_enabled": lidar_enabled,
            "odometry_source": odometry_source,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
            DeclareLaunchArgument("lidar_enabled", default_value=lidar_enabled),
            DeclareLaunchArgument("odometry_source", default_value="encoders"),
            node_rviz,
            launch_p3dx_description,
        ]
    )
