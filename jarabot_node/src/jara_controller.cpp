#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jarabot_interfaces/msg/cmd.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JaraController : public rclcpp::Node
{
public:
    JaraController() : Node("jara_controller"),
                       keyboard_linear_gain_((1750 - 1500) / 0.5),
                       keyboard_angular_gain_((1750 - 1500) / 1.0),
                       target_linear_input_(1500),
                       target_angular_input_(1500),
                       current_linear_input_(1500),
                       current_angular_input_(1500),
                       button_joy_(false),
                       button_movebase_(false),
                       keyboard_(false)

    {
        // this->declare_parameter("linear_gain", 500.0);
        // this->declare_parameter("angular_gain", 500.0);
        this->declare_parameter("wheel_radius", 0.035);
        this->declare_parameter("wheel_base", 0.220);
        this->declare_parameter("loop_rate", 20);

        // linear_gain_ = this->get_parameter("linear_gain").get_parameter_value().get<double>();
        // angular_gain_ = this->get_parameter("angular_gain").get_parameter_value().get<double>();
        wheel_radius_ = this->get_parameter("wheel_radius").get_parameter_value().get<double>();
        wheel_base_ = this->get_parameter("wheel_base").get_parameter_value().get<double>();
        loop_rate_ = this->get_parameter("loop_rate").get_parameter_value().get<int>();

        smoother_step_ = (2000 - 1660) / (loop_rate_ * 0.5);

        // RCLCPP_INFO(this->get_logger(), "%f %f %f %f %i", linear_gain_, angular_gain_, wheel_radius_, wheel_base_, loop_rate_);

        cmd_pub_ = this->create_publisher<jarabot_interfaces::msg::Cmd>("/cmd", loop_rate_);

        mainloop_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / loop_rate_), std::bind(&JaraController::main_loop, this));

        joy_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/joy/cmd_vel", loop_rate_, std::bind(&JaraController::joy_cmd_vel_callback, this, _1));
        movebase_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", loop_rate_, std::bind(&JaraController::movebase_cmd_vel_callback, this, _1));
        key_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/keyboard/cmd_vel", loop_rate_, std::bind(&JaraController::keyboard_cmd_vel_callback, this, _1));
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", loop_rate_, std::bind(&JaraController::joy_callback, this, _1));
    }

private:
    void main_loop()
    {
        // RCLCPP_ERROR(this->get_logger(), "main_loop");
        // if ((!button_joy_) && (!button_movebase_) && (!keyboard_))
        {
            bool shouldSendCommand = false;
            if (current_linear_input_<target_linear_input_)
            {
                if( (target_linear_input_-current_linear_input_) > smoother_step_)
                {
                    current_linear_input_ += smoother_step_;
                    shouldSendCommand = true;
                }
                else
                {
                    current_linear_input_ = target_linear_input_;
                    shouldSendCommand = true;
                }
            }
            else if (current_linear_input_>target_linear_input_)
            {
                if( (current_linear_input_-target_linear_input_) > smoother_step_)
                {
                    current_linear_input_ -= smoother_step_;
                    shouldSendCommand = true;
                }
                else
                {
                    current_linear_input_ = target_linear_input_;
                    shouldSendCommand = true;
                }
            }

            if (current_angular_input_<target_angular_input_)
            {
                if( (target_angular_input_-current_angular_input_) > smoother_step_)
                {
                    current_angular_input_ += smoother_step_;
                    shouldSendCommand = true;
                }
                else
                {
                    current_angular_input_ = target_angular_input_;
                    shouldSendCommand = true;
                }
            }
            else if (current_angular_input_>target_angular_input_)
            {
                if( (current_angular_input_-target_angular_input_) > smoother_step_)
                {
                    current_angular_input_ -= smoother_step_;
                    shouldSendCommand = true;
                }
                else
                {
                    current_angular_input_ = target_angular_input_;
                    shouldSendCommand = true;
                }
            }

            if (shouldSendCommand)
            {
                auto cmd_msg = jarabot_interfaces::msg::Cmd();
                cmd_msg.linear_input = current_linear_input_;
                cmd_msg.angular_input = current_angular_input_;
                cmd_pub_->publish(cmd_msg);
            }
        }
    }

    void joy_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (button_joy_ && !button_movebase_)
        {
            publish_cmd_msg(msg->linear.x, msg->angular.z);
        }
    }

    void movebase_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "get movebase cmd vel");
        // if (!button_joy_ && button_movebase_)
        {
            double linear_input = msg->linear.x;
            double angular_input = msg->angular.z;

            if (linear_input > 0)
            {
                linear_input = 1500.0 + 170.0 + linear_input*((500-170)/0.26);
            }
            else if (linear_input < 0)
            {
                linear_input = 1500.0 - 170.0 + linear_input*((500-170)/0.26);
            }
            else
            {
                linear_input = 1500.0;
            }

            if (angular_input > 0)
            {
                angular_input = 1500.0 - 170.0 + -1.0*( angular_input*((500-170)/1.0) );
            }
            else if (angular_input < 0)
            {
                angular_input = 1500.0 + 170.0 + -1.0*( angular_input*((500-170)/1.0) );
            }
            else
            {
                angular_input = 1500.0;
            }
            publish_cmd_msg(linear_input, angular_input);
        }
    }

    void keyboard_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // keyboard_ = true;
        // RCLCPP_INFO(this->get_logger(), "get key cmd vel");
        publish_cmd_msg((keyboard_linear_gain_ * msg->linear.x) + 1500.0, (-keyboard_angular_gain_ * msg->angular.z) + 1500.0);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        button_joy_ = joy_msg->buttons[4] == 1;
        button_movebase_ = joy_msg->buttons[5] == 1;
    }

    void publish_cmd_msg(double vl, double va)
    {
        double linear_input = vl;
        double angular_input = va;

        // linear_input += 1500.0;
        // angular_input += 1500.0;

        if (linear_input > 1999.0)
            linear_input = 1999.0;
        else if (linear_input < 1001.0)
            linear_input = 1001.0;

        if (angular_input > 1999.0)
            angular_input = 1999.0;
        else if (angular_input < 1001.0)
            angular_input = 1001.0;

        auto cmd_msg = jarabot_interfaces::msg::Cmd();
        cmd_msg.linear_input = static_cast<int>(std::round(linear_input));
        cmd_msg.angular_input = static_cast<int>(std::round(angular_input));

        // cmd_pub_->publish(cmd_msg);

        target_linear_input_ = static_cast<int>(std::round(linear_input));
        target_angular_input_ = static_cast<int>(std::round(angular_input));
    }

private:
    rclcpp::Publisher<jarabot_interfaces::msg::Cmd>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr movebase_cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr mainloop_timer_;
    double keyboard_linear_gain_, keyboard_angular_gain_, wheel_radius_, wheel_base_;
    int loop_rate_, target_linear_input_, target_angular_input_, current_linear_input_, current_angular_input_, smoother_step_;
    bool button_joy_, button_movebase_, keyboard_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraController>());
    rclcpp::shutdown();
    return 0;
}
