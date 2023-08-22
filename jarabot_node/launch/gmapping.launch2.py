from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='slam_gmapping',
            name='slam_gmapping',
            executable='slam_gmapping'
        )
    ])
