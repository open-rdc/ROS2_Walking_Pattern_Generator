#include "rclcpp/rclcpp.hpp"
#include "walking_stabilization_controller/WalkingStabilizationController.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor exe;
  // exe.add_node(std::make_shared<walking_stabilization_controller::WalkingStabilizationController>());
  // exe.spin();
  auto node = std::make_shared<robot_manager::WalkingStabilizationController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

/* Reference
  https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

*/