#include "rclcpp/rclcpp.hpp"
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walking_pattern_generator::WalkingPatternGenerator>());
  rclcpp::shutdown();

  return 0;
}