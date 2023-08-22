from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_launch = launch_ros.actions.IncludeLaunchDescription(package='jarabot_node', launch='bringup_launch.py')
    bt_navigator_launch = launch_ros.actions.IncludeLaunchDescription(package='jarabot_node', launch='bt_navigator_launch.py') 

    explore_launch = launch_ros.actions.IncludeLaunchDescription(package='jarabot_node', launch='explore_launch.py') 

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('jarabot_node'),
        'config')

    return LaunchDescription([
        bringup_launch,
        bt_navigator_launch,
        explore_launch,
        Node(
            package='gmapping',
            name='slam_gmapping',
            executable='slam_gmapping',
            output="screen"
        ),
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2'
        )
    ])


# <!-- -*- mode: XML -*- -->
# <launch>

#   <include file="$(find dronebot23)/launch/bringup.launch" />

#   <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
#     <param name="base_frame" value="base_footprint" />
#     <param name="odom_frame" value="odom" />
#     <param name="map_frame" value="map" />
#     <rosparam command="load" file="$(find dronebot23)/config/gmapping.yaml" />
#   </node>

#   <include file="$(find dronebot23)/launch/include/move_base.launch"/>

#   <include file="$(find explore_lite)/launch/explore.launch"/>

#   <node pkg="rviz" type="rviz" name="rviz" />
#   <!-- <node name="rviz" pkg="rviz" type="rviz" args="-d $(find dronebot23)/rviz/explore.rviz" /> -->

# </launch>
