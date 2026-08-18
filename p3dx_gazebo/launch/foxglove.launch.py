#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Whitelist topics
DEFAULT_TOPIC_WHITELIST = [
    r"^/tf$",
    r"^/tf_static$",
    r"^(/[^/]+)?/robot_description$",
    r"^(/[^/]+)?/scan$",
    r"^(/[^/]+)?/RosAria/odom$",
    r"^(/[^/]+)?/odometry/filtered$",
    r"^(/[^/]+)?/map$",
    r"^(/[^/]+)?/(global|local)_costmap/costmap$",
    r"^(/[^/]+)?/plan$",
    r"^(/[^/]+)?/local_plan$",
    r"^(/[^/]+)?/cmd_vel$",
    r"^(/[^/]+)?/(nav2/cmd_vel|RosAria/cmd_vel)$",
    r"^(/[^/]+)?/goal_pose$",
    r"^(/[^/]+)?/waypoints$",
    r"^(/[^/]+)?/(visualization_marker|visualization_marker_array)$",
    r"^(/[^/]+)?/twist_marker_server/.*$",
    r"^/rosout$",
]


def generate_launch_description():
    port = LaunchConfiguration("port", default="8765")
    address = LaunchConfiguration("address", default="0.0.0.0")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    topic_whitelist = LaunchConfiguration(
        "topic_whitelist", default=str(DEFAULT_TOPIC_WHITELIST)
    )
    send_buffer_limit = LaunchConfiguration("send_buffer_limit", default="10000000")

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": port,
                "address": address,
                "use_sim_time": use_sim_time,
                "topic_whitelist": topic_whitelist,
                "send_buffer_limit": send_buffer_limit,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value=port,
                description="WebSocket port foxglove_bridge listens on.",
            ),
            DeclareLaunchArgument(
                "address",
                default_value=address,
                description="Interface to bind. Must stay 0.0.0.0 (all interfaces) "
                "for a remote Mac to reach it - binding localhost is the most "
                "common reason remote viewing silently fails.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use the Gazebo /clock. Defaults to true here "
                "(unlike foxglove_bridge's own default of false) so Foxglove "
                "shows correct simulated timestamps.",
            ),
            DeclareLaunchArgument(
                "topic_whitelist",
                default_value=topic_whitelist,
                description="Regex list of topics to forward. Defaults to a "
                "curated set for navigation debugging over WiFi (excludes the "
                "voxel grid, camera/image topics, and other high-rate data). "
                "Pass topic_whitelist:=\"['.*']\" to forward everything when "
                "the link can take it.",
            ),
            DeclareLaunchArgument(
                "send_buffer_limit",
                default_value=send_buffer_limit,
                description="Max bytes buffered per client connection before "
                "foxglove_bridge drops messages. Lower this on a poor link.",
            ),
            foxglove_bridge,
        ]
    )
