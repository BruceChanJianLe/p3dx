#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    this_package_path = get_package_share_directory("p3dx_gazebo")
    p3dx_description_path = get_package_share_directory("p3dx_description")
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    world_name = LaunchConfiguration("world_name", default="warehouse_world")
    world_file = LaunchConfiguration(
        "world_file", default=[join(this_package_path, "worlds/"), world_name, ".sdf"]
    )
    x = LaunchConfiguration("x", default=12.86)
    y = LaunchConfiguration("y", default=-9.48)
    yaw = LaunchConfiguration("yaw", default=0.0)
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    lidar_enabled = LaunchConfiguration("lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "empty_world.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_name": world_name,
            "world_file": world_file,
        }.items(),
    )

    # Must have for multi robot
    # Handle the general gazebo topics
    gz_ros2_bridge_general = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_general",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ],
    )

    # pioneer a
    spawn_p3dx_pioneer_a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "spawn_p3dx.launch.py")
        ),
        launch_arguments={
            "robot_namespace": "pioneer_a",
            "world_name": world_name,
            "camera_enabled": camera_enabled,
            "lidar_enabled": lidar_enabled,
            "odometry_source": odometry_source,
            "x": x,
            "y": y,
            "yaw": yaw,
        }.items(),
    )

    # pioneer b
    spawn_p3dx_pioneer_b = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "spawn_p3dx.launch.py")
        ),
        launch_arguments={
            "robot_namespace": "pioneer_b",
            "world_name": world_name,
            "camera_enabled": camera_enabled,
            "lidar_enabled": lidar_enabled,
            "odometry_source": odometry_source,
            "x": "10.0",
            "y": y,
            "yaw": yaw,
        }.items(),
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH", value=join(this_package_path, "worlds")
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH", value=join(this_package_path, "models")
            ),
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument("world_name", default_value=world_name),
            DeclareLaunchArgument("world_file", default_value=world_file),
            DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
            DeclareLaunchArgument("lidar_enabled", default_value=lidar_enabled),
            DeclareLaunchArgument("odometry_source", default_value="encoders"),
            DeclareLaunchArgument("x", default_value=x),
            DeclareLaunchArgument("y", default_value=y),
            DeclareLaunchArgument("yaw", default_value=yaw),
            gz_sim,
            spawn_p3dx_pioneer_a,
            spawn_p3dx_pioneer_b,
            gz_ros2_bridge_general,
        ]
    )
