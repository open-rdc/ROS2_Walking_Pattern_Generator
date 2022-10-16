#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace test1
{
    class Test1_Sub : public rclcpp::Node {
        public:
            rclcpp::NodeOptions &options = rclcpp::NodeOptions();
        private:
            void test1_callback(const std_msgs::msg::String::SharedPtr sub_data);
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test1_subscription;

    };
}
