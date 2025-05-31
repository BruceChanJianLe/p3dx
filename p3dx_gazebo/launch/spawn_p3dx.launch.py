#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution, PythonExpression, EqualsSubstitution, NotEqualsSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

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
  robot_namespace = LaunchConfiguration("robot_namespace", default="")

  robot_state_publisher_namespace = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=robot_namespace,
    parameters=[
      {
      'frame_prefix': [robot_namespace, TextSubstitution(text='/')],
      'robot_description': Command( \
      ['xacro ', join(p3dx_description_path, 'urdf/p3dx/pioneer3dx.xacro'),
      ' camera_enabled:=', camera_enabled,
      ' lidar_enabled:=', lidar_enabled,
      ' odometry_source:=', odometry_source,
      ' robot_namespace:=', robot_namespace,
      ])}],
    remappings=[
        ('/joint_states', 'joint_states'),
    ],
    condition=IfCondition(
        NotEqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
    )
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=robot_namespace,
    parameters=[
      {
      'robot_description': Command( \
      ['xacro ', join(p3dx_description_path, 'urdf/p3dx/pioneer3dx.xacro'),
      ' camera_enabled:=', camera_enabled,
      ' lidar_enabled:=', lidar_enabled,
      ' odometry_source:=', odometry_source,
      ' robot_namespace:=', robot_namespace,
      ])}],
    remappings=[
        ('/joint_states', 'joint_states'),
    ],
    condition=IfCondition(
        EqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
    )
  )

  gz_spawn_entity_namespace = Node(
    package="ros_gz_sim",
    executable="create",
    namespace=robot_namespace,
    arguments=[
      "-topic", "robot_description",
      "-name", robot_namespace,
      "-allow_renaming", "true",
      "-z", "0.28",
      "-x", x,
      "-y", y,
      "-Y", yaw
    ],
    condition=IfCondition(
        NotEqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
    )
  )

  gz_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    namespace=robot_namespace,
    arguments=[
      "-topic", "robot_description",
      "-name", "pioneer",
      "-allow_renaming", "true",
      "-z", "0.28",
      "-x", x,
      "-y", y,
      "-Y", yaw
    ],
    condition=IfCondition(
        EqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
    )
  )

  gz_ros2_bridge_namespace = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    namespace=robot_namespace,
    arguments=[
        PathJoinSubstitution([robot_namespace, "RosAria/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"]),
        PathJoinSubstitution([robot_namespace, "RosAria/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry"]),
        PathJoinSubstitution([robot_namespace, "scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"]),
        PathJoinSubstitution([robot_namespace, "camera@sensor_msgs/msg/Image[ignition.msgs.Image"]),
        PathJoinSubstitution([robot_namespace, "camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"]),
        PathJoinSubstitution(["/world", world_name , "model", robot_namespace, "joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"])
      ],
    remappings=[
        (PathJoinSubstitution(['/', robot_namespace, robot_namespace, 'RosAria/cmd_vel']),      'RosAria/cmd_vel'),
        (PathJoinSubstitution(['/', robot_namespace, robot_namespace, 'RosAria/odom']),         'RosAria/odom'),
        (PathJoinSubstitution(['/', robot_namespace, robot_namespace, 'scan']),                 'scan'),
        (PathJoinSubstitution(['/', robot_namespace, robot_namespace, 'camera']),               'camera'),
        (PathJoinSubstitution(['/', robot_namespace, robot_namespace, 'camera/camera_info']),   'camera/camera_info'),
        (PathJoinSubstitution(['/world', world_name, 'model', robot_namespace, 'joint_state']), 'joint_states'),
      ],
    condition=IfCondition(
        NotEqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
    )
  )

  # # Must have for multi robot, include this in the multi robot launch file
  # # Handle the general gazebo topics
  # gz_ros2_bridge_general_namespace = Node(
  #   package="ros_gz_bridge",
  #   executable="parameter_bridge",
  #   name="ros_gz_bridge_general",
  #   arguments=[
  #       "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
  #       "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
  #     ],
  #   condition=IfCondition(
  #       NotEqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
  #   )
  # )

  gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        "/RosAria/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        "/RosAria/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/scan0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan3@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan4@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan5@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan6@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan7@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/scan8@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        PathJoinSubstitution(["/world", world_name , "model/pioneer/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"])
      ],
    remappings=[
        # ('/RosAria/cmd_vel', '/cmd_vel'),
        # ('/RosAria/odom', '/odom'),
        (PathJoinSubstitution(['/world', world_name, 'model/pioneer/joint_state']), 'joint_states'),
      ],
    condition=IfCondition(
        EqualsSubstitution(LaunchConfiguration('robot_namespace'), "")
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument("world_name", default_value=world_name),
    DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
    DeclareLaunchArgument("lidar_enabled", default_value = lidar_enabled),
    DeclareLaunchArgument("odometry_source", default_value=odometry_source),
    DeclareLaunchArgument("x", default_value="0.0"),
    DeclareLaunchArgument("y", default_value="0.0"),
    DeclareLaunchArgument("yaw", default_value="0.0"),
    DeclareLaunchArgument("robot_namespace", default_value=""),
    robot_state_publisher,
    robot_state_publisher_namespace,
    gz_spawn_entity,
    gz_spawn_entity_namespace,
    gz_ros2_bridge,
    gz_ros2_bridge_namespace,
    # gz_ros2_bridge_general_namespace,
  ])
from launch_ros.actions import Node
