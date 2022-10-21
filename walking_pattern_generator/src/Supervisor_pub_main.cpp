#include "rclcpp/rclcpp.hpp"

#include "walking_pattern_generator/Supervisor_pub.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SupervisorPub>());
    rclcpp::shutdown();

    return 0;
}