#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "Walking_Pattern_Generator/WalkingPatternGenerator.hpp"

namespace walking_pattern_generator
{
  using namespace WalkingPatternGenerator;
  
  void callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // callback
  }

  WalkingPatternGenerator(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
  ) {
    // walking_pattern_generator
  }
}