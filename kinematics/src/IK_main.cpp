#include "rclcpp/rclcpp.hpp"
#include "kinematics/IK.hpp"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kinematics::IK>());
  rclcpp::shutdown();

  return 0;
}