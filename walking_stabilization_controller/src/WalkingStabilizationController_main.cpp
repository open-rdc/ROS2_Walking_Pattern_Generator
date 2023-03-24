#include "rclcpp/rclcpp.hpp"
#include "walking_stabilization_controller/WalkingStabilizationController.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walking_stabilization_controller::WalkingStabilizationController>());
  rclcpp::shutdown();

  return 0;
}