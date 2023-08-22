#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "jarabot_interfaces/msg/cmd.hpp"
#include "jarabot_interfaces/msg/ecd.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JaraDriver : public rclcpp::Node
{
public:
    JaraDriver() : Node("jara_driver"),
                   linear_input_(1500),
                   angular_input_(1500)
    {
        this->declare_parameter("loop_rate", 20);
        loop_rate_ = this->get_parameter("loop_rate").get_parameter_value().get<int>();

        ecd_pub_ = this->create_publisher<jarabot_interfaces::msg::Ecd>("/ecd", loop_rate_);
        serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/serial_write", loop_rate_);

        cmd_sub_ = create_subscription<jarabot_interfaces::msg::Cmd>(
            "/cmd", loop_rate_, std::bind(&JaraDriver::cmd_callback, this, _1));
        serial_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/serial_read", loop_rate_, std::bind(&JaraDriver::serial_callback, this, _1));
    }

private:
    void cmd_callback(const jarabot_interfaces::msg::Cmd::SharedPtr msg)
    {
        auto serial_msg = std_msgs::msg::UInt8MultiArray();
        std::string str = std::string("pwm,") + std::to_string(msg->angular_input) + std::string(",") + std::to_string(msg->linear_input) + std::string(" \n");

        serial_msg.data.reserve(str.size());
        for (char c : str)
        {
            serial_msg.data.push_back(static_cast<uint8_t>(c));
        }

        serial_pub_->publish(serial_msg);
    }

    void serial_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        // Convert UInt8MultiArray data to std::string
        std::string str;
        for (const auto &byte : msg->data)
        {
            str.push_back(static_cast<char>(byte));
        }
        // RCLCPP_INFO(this->get_logger(), "%s", str.c_str());

        // parse string
        std::istringstream iss(str);
        std::string line;

        // while (std::getline(iss, line, '\n'))
        // {
        std::getline(iss, line, '\n');
        std::getline(iss, line, '\n');
        // ROS_INFO_STREAM("curr_line : " << line);
        size_t comma_pos = line.find(',');

        // Check if there is a comma in the line
        if (comma_pos != std::string::npos)
        {
            // Get the left and right parts of the line
            std::string left_data = line.substr(0, comma_pos);
            std::string right_data = line.substr(comma_pos + 1);
            // ROS_INFO_STREAM("comma_pos : " << comma_pos);
            // ROS_INFO_STREAM("left_data : " << left_data);
            // Convert left_data and right_data to integers
            try
            {
                auto ecd_msg = jarabot_interfaces::msg::Ecd();
                ecd_msg.header.stamp = this->get_clock()->now();
                ecd_msg.left_encoder_val = std::stoi(left_data);
                ecd_msg.right_encoder_val = std::stoi(right_data);
                ecd_pub_->publish(ecd_msg);
            }
            catch (const std::invalid_argument &e)
            {
                // continue;
            }
        }
        else
        {
            // continue;
        }
        // }
    }

private:
    rclcpp::Publisher<jarabot_interfaces::msg::Ecd>::SharedPtr ecd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;
    rclcpp::Subscription<jarabot_interfaces::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub_;
    int loop_rate_, linear_input_, angular_input_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraDriver>());
    rclcpp::shutdown();
    return 0;
}
