from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='nav2_amcl',
            name='amcl',
            executable='amcl',
            output="screen",
            parameters=[
                {"min_particles":"500"},
                {"max_particles":"3000"},
                {"kld_err":"0.02"},
                {"update_min_d":"0.20"},
                {"update_min_a":"0.20"},
                {"resample_interval":"1"},
                {"transform_tolerance":"0.5"},
                {"recovery_alpha_slow":"0.00"},
                {"recovery_alpha_fast":"0.00"},
                {"initial_pose_x":"$(arg initial_pose_x)"},
                {"initial_pose_y":"$(arg initial_pose_y)"},
                {"initial_pose_a":"$(arg initial_pose_a)"},
                {"gui_publish_rate":"50.0"},
                {"laser_max_range":"3.5"},
                {"laser_max_beams":"180"},
                {"laser_z_hit":"0.5"},
                {"laser_z_short":"0.05"},
                {"laser_z_max":"0.05"},
                {"laser_z_rand":"0.5"},
                {"laser_sigma_hit":"0.2"},
                {"laser_lambda_short":"0.1"},
                {"laser_likelihood_max_dist":"2.0"},
                {"laser_model_type":"likelihood_field"},
                {"odom_model_type":"diff"},
                {"odom_alpha1":"0.1"},
                {"odom_alpha2":"0.1"},
                {"odom_alpha3":"0.1"},
                {"odom_alpha4":"0.1"},
                {"odom_frame_id":"odom"},
                {"base_frame_id":"base_footprint"}
            ]
        )
    ])


# <launch>
#   <!-- Arguments -->
#   <!-- <arg name="scan_topic"     default="scan"/> -->
#   <arg name="initial_pose_x" default="0.0"/>
#   <arg name="initial_pose_y" default="0.0"/>
#   <arg name="initial_pose_a" default="0.0"/>

#   <!-- AMCL -->
#   <node pkg="amcl" type="amcl" name="amcl">

#     <param name="min_particles"             value="500"/>
#     <param name="max_particles"             value="3000"/>
#     <param name="kld_err"                   value="0.02"/>
#     <param name="update_min_d"              value="0.20"/>
#     <param name="update_min_a"              value="0.20"/>
#     <param name="resample_interval"         value="1"/>
#     <param name="transform_tolerance"       value="0.5"/>
#     <param name="recovery_alpha_slow"       value="0.00"/>
#     <param name="recovery_alpha_fast"       value="0.00"/>
#     <param name="initial_pose_x"            value="$(arg initial_pose_x)"/>
#     <param name="initial_pose_y"            value="$(arg initial_pose_y)"/>
#     <param name="initial_pose_a"            value="$(arg initial_pose_a)"/>
#     <param name="gui_publish_rate"          value="50.0"/>

#     <!-- <remap from="scan"                      to="$(arg scan_topic)"/> -->
#     <param name="laser_max_range"           value="3.5"/>
#     <param name="laser_max_beams"           value="180"/>
#     <param name="laser_z_hit"               value="0.5"/>
#     <param name="laser_z_short"             value="0.05"/>
#     <param name="laser_z_max"               value="0.05"/>
#     <param name="laser_z_rand"              value="0.5"/>
#     <param name="laser_sigma_hit"           value="0.2"/>
#     <param name="laser_lambda_short"        value="0.1"/>
#     <param name="laser_likelihood_max_dist" value="2.0"/>
#     <param name="laser_model_type"          value="likelihood_field"/>

#     <param name="odom_model_type"           value="diff"/>
#     <param name="odom_alpha1"               value="0.1"/>
#     <param name="odom_alpha2"               value="0.1"/>
#     <param name="odom_alpha3"               value="0.1"/>
#     <param name="odom_alpha4"               value="0.1"/>
#     <param name="odom_frame_id"             value="odom"/>
#     <param name="base_frame_id"             value="base_footprint"/>

#   </node>
# </launch>
