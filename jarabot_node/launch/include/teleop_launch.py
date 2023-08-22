from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='joy',
            name='joy_node',
            executable='joy_node',
            output="screen",
            parameters=[
                {"dev":"/dev/input/myjoy"},
                {"autorepeat_rate":"20"}
            ]
        ),
        Node(
            package='teleop_twist_joy',
            name='teleop_twist_joy',
            executable='teleop_node',
            output="screen",
            remap=[
                {"cmd_vel", "/joy/cmd_vel"}
            ],
            parameters=[
                {"enable_button":"4"}
            ]
        ),
        Node(
            package='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output="screen",
            prefix='gnome-terminal --',
            remappings=[
                ('/cmd_vel', '/keyboard/cmd_vel')
            ]
        ),
    ])
