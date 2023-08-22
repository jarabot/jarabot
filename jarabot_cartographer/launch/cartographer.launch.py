
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    jarabot_cartographer_prefix = get_package_share_directory('jarabot_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  jarabot_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='jarabot_rplidar_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('jarabot_cartographer'),
                                   'rviz', 'jarabot_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='config 파일의 전체 경로'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='cartographer 설정에 사용하는 lua 파일 이름'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='simulation(Gazebo) clock 사용여부'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='occupancy grid의 각 cell의 resolution'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
