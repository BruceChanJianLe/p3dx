#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

  this_package_path = get_package_share_directory("p3dx_gazebo")
  p3dx_description_path = get_package_share_directory("p3dx_description")
  world_name = LaunchConfiguration("world_name", default="warehouse_world")
  x = LaunchConfiguration("x")
  y = LaunchConfiguration("y")
  yaw = LaunchConfiguration("yaw")
  camera_enabled = LaunchConfiguration("camera_enabled", default=True)
  lidar_enabled = LaunchConfiguration("lidar_enabled", default=True)
  odometry_source = LaunchConfiguration("odometry_source", default="encoders")

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    parameters=[
      {'robot_description': Command( \
      ['xacro ', join(p3dx_description_path, 'urdf/p3dx/pioneer3dx.xacro'),
      ' camera_enabled:=', camera_enabled,
      ' lidar_enabled:=', lidar_enabled,
      ' odometry_source:=', odometry_source,
      ])}],
    remappings=[
        ('/joint_states', 'pioneer/joint_states'),
    ]
  )

  gz_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
      "-topic", "/robot_description",
      "-name", "pioneer",
      "-allow_renaming", "true",
      "-z", "0.28",
      "-x", x,
      "-y", y,
      "-Y", yaw
    ]
  )

  gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/RosAria/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/RosAria/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        # Note that world must be exactly the same as world name
        PathJoinSubstitution(["/world", world_name , "model/pioneer/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"])
      ],
    remappings=[
        # ('/RosAria/cmd_vel', '/cmd_vel'),
        # ('/RosAria/odom', '/odom'),
        (PathJoinSubstitution(['/world', world_name, 'model/pioneer/joint_state']), 'pioneer/joint_states'),
      ]
   )

  return LaunchDescription([
    DeclareLaunchArgument("world_name", default_value=world_name),
    DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
    DeclareLaunchArgument("lidar_enabled", default_value = lidar_enabled),
    DeclareLaunchArgument("odometry_source", default_value=odometry_source),
    DeclareLaunchArgument("x", default_value="0.0"),
    DeclareLaunchArgument("y", default_value="0.0"),
    DeclareLaunchArgument("yaw", default_value="0.0"),
    robot_state_publisher,
    gz_spawn_entity,
    gz_ros2_bridge
  ])
from launch_ros.actions import Node
