import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('loop_rate', default_value='20'),
        DeclareLaunchArgument('wheel_radius', default_value='0.035'),
        DeclareLaunchArgument('wheel_base', default_value='0.220'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/mylidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'base_scan',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            # output='screen',
            arguments=['--x', '0.025', '--z', '0.225', '--frame-id', 'base_link', '--child-frame-id', 'base_scan'],
            # arguments=['0.025', '0.0', '0.225', '0.0', '0.0', '0.0', '1.0', 'base_link', 'base_scan'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            # output='screen',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
            # arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'base_footprint'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('jarabot_node'),
                    'launch',
                    'include',
                    'serial_driver.launch.py'
                ])
            ]),
        ),
        Node(
            package='jarabot_node',
            name='jara_controller',
            executable='jara_controller',
            output='screen',
            parameters=[{
                # 'linear_gain': 1000.0,
                # 'angular_gain': 500.0,
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'loop_rate': LaunchConfiguration('loop_rate'),
            }],
        ),
        Node(
            package='jarabot_node',
            name='jara_driver',
            executable='jara_driver',
            parameters=[{
                'loop_rate': LaunchConfiguration('loop_rate'),
            }],
        ),
        Node(
            package='jarabot_node',
            name='jara_odometry',
            executable='jara_odometry',
            parameters=[{
                'loop_rate': LaunchConfiguration('loop_rate'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'encoder_resolution': 1200,
            }],
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('jarabot_node'),
        #             'launch',
        #             'include',
        #             'teleo_launch.py'
        #         ])
        #     ]),
        # )
    ])

