#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "jarabot_interfaces/msg/ecd.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JaraOdometry : public rclcpp::Node
{
public:
    JaraOdometry() : Node("jara_odometry"),

                     x_(0.0),
                     y_(0.0),
                     theta_(0.0),
                     v_x_(0.0),
                     v_y_(0.0),
                     v_theta_(0.0),
                     prev_l_ecd_(0),
                     prev_r_ecd_(0)
    {
        this->declare_parameter("loop_rate", 20);
        this->declare_parameter("encoder_resolution", 1200);
        this->declare_parameter("wheel_radius", 0.035);
        this->declare_parameter("wheel_base", 0.220);

        loop_rate_ = this->get_parameter("loop_rate").get_parameter_value().get<int>();
        encoder_resolution_ = this->get_parameter("encoder_resolution").get_parameter_value().get<int>();
        wheel_radius_ = this->get_parameter("wheel_radius").get_parameter_value().get<double>();
        wheel_base_ = this->get_parameter("wheel_base").get_parameter_value().get<double>();

        last_time_ = this->now();

        odom_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", loop_rate_);

        ecd_sub_ = create_subscription<jarabot_interfaces::msg::Ecd>(
            "/ecd", 20, std::bind(&JaraOdometry::ecd_callback, this, _1));
    }

private:
    void ecd_callback(const jarabot_interfaces::msg::Ecd::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        rclcpp::Time ecd_time = msg->header.stamp;
        int l_ecd = msg->left_encoder_val;
        int r_ecd = msg->right_encoder_val;

        // calculate change of encoder value
        int del_l_ecd = l_ecd - prev_l_ecd_;
        int del_r_ecd = r_ecd - prev_r_ecd_;

        // calculate displacement
        double left_wheel_distance = wheel_radius_ * del_l_ecd * 2 * M_PI / encoder_resolution_;
        double right_wheel_distance = wheel_radius_ * del_r_ecd * 2 * M_PI / encoder_resolution_;
        double delta_distance = 0.5 * (left_wheel_distance + right_wheel_distance);
        double delta_theta = (right_wheel_distance - left_wheel_distance) / wheel_base_;

        // update position
        x_ += delta_distance * cos(theta_);
        y_ += delta_distance * sin(theta_);
        theta_ += delta_theta;

        // update position
        v_x_ = delta_distance / (ecd_time - last_time_).seconds();
        v_theta_ = delta_theta / (ecd_time - last_time_).seconds();

        // since all odometry is 6DOF we'll need a quaternion created from yaw
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        q.normalize();
        geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

        // first, we'll publish the transform over tf
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform
        odom_broadcaster_->sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        // set the position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v_x_;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = v_theta_;

        // publish the message
        odom_pub_->publish(odom);

        last_time_ = ecd_time;
        prev_l_ecd_ = l_ecd;
        prev_r_ecd_ = r_ecd;
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<jarabot_interfaces::msg::Ecd>::SharedPtr ecd_sub_;
    rclcpp::Time last_time_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    double wheel_radius_, wheel_base_, x_, y_, theta_, v_x_, v_y_, v_theta_;
    int loop_rate_, encoder_resolution_, prev_l_ecd_, prev_r_ecd_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraOdometry>());
    rclcpp::shutdown();
    return 0;
}
