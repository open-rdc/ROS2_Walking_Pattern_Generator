#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "walking_pattern_generator/test1_sub.hpp"

namespace test1
{
    Test1_Sub::Test1_Sub(const rclcpp::NodeOptions &options)
    : Node("Test1_Sub", options) {
        test1_subscription = this->create_subscription<std_msgs::msg::String>("test", rclcpp::QoS(10), std::bind(&Test1_Sub::test1_callback, this, std::placeholders::_1));
    }

    void Test1_Sub::test1_callback(const std_msgs::msg::String::SharedPtr sub_data) {
        RCLCPP_INFO(this->get_logger(), "Sub: %s", sub_data->data.c_str());
    }
}