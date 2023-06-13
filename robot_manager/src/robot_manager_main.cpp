#include "rclcpp/rclcpp.hpp"
#include "robot_manager/robot_manager.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_manager::RobotManager>();
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node);
  exe.spin();
  // rclcpp::spin(std::make_shared<robot_manager::RobotManager>());
  rclcpp::shutdown();

  return 0;
}

/* Reference
  https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

*/