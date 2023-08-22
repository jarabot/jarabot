import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch.events import matches_action
# from launch.events import matches_node_name
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('jarabot_node'), 'config')
    node_name = 'serial_bridge_node'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            config_dir, 'serial_driver.yaml'),
        description='File path to the ROS2 parameters file to use'
    )

    bridge_node = LifecycleNode(
        package='serial_driver',
        executable='serial_bridge',
        name=node_name,
        namespace=TextSubstitution(text=''),
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=bridge_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name(node_name),
                        transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        params_declare,
        bridge_node,
        configure_event_handler,
        activate_event_handler,
        shutdown_event_handler,
    ])
