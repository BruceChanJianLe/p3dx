import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, EqualsSubstitution, NotEqualsSubstitution, AndSubstitution, NotSubstitution
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    robot_namespace = LaunchConfiguration("robot_namespace", default="")
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    remappings = [
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata')
            ]

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("p3dx_navigation"),
                                   'config', 'slam_2d.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Top-level namespace')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
            'use_sim_time': use_sim_time,
            'use_lifecycle_manager': use_lifecycle_manager,
            }

    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={'<tf_prefix>': (robot_namespace, '/') },
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration('robot_namespace'), "")),
        )

    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={'<tf_prefix>': (robot_namespace) },
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('robot_namespace'), "")),
        )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=slam_params_file,
            root_key=robot_namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[configured_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_namespace,
        remappings=remappings,
        output='screen')

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
