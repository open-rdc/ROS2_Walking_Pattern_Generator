#include "rclcpp/rclcpp.hpp"
#include "kinematics/FK.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kinematics::FK>());
  rclcpp::shutdown();
  
  return 0;
}
