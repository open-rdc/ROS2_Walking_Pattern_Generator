#include "rclcpp/rclcpp.hpp"
#include "robot_manager/robot_manager.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor exe;
  // exe.add_node(std::make_shared<robot_manager::RobotManager>());
  // exe.spin();
  rclcpp::spin(std::make_shared<robot_manager::RobotManager>());
  rclcpp::shutdown();

  return 0;
}