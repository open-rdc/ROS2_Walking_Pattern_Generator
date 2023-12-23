#include "robot_manager/robot_manager.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<robot_manager::RobotManager>(opt));
  rclcpp::shutdown();

  return 0;
}

// int main(int argc, char* argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<robot_manager::RobotManager>();
//   rclcpp::spin(node->get_node_base_interface());
//   rclcpp::shutdown();

//   return 0;
// }