#include "rclcpp/rclcpp.hpp"
#include "old_kinematics/FK.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<old_kinematics::FK>());
  rclcpp::shutdown();
  
  return 0;
}
