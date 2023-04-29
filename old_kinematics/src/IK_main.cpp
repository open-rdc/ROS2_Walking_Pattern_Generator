#include "rclcpp/rclcpp.hpp"
#include "old_kinematics/IK.hpp"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<old_kinematics::IK>());
  rclcpp::shutdown();

  return 0;
}