#include "rclcpp/rclcpp.hpp"
#include "walking_stabilization_controller/WalkingStabilizationController.hpp"
#include "robot_manager/robot_manager.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(std::make_shared<robot_manager::WalkingStabilizationController>());
    exec.add_node(std::make_shared<robot_manager::RobotManager>());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}