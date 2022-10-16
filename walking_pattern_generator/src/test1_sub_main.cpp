#include <rclcpp/rclcpp.hpp>

#include "walking_pattern_generator/test1_sub.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<test1::Test1_Sub>());
    rclcpp::shutdown();
    
    return 0;
}
