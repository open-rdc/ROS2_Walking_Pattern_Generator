#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "/*PackageName*//msg/ToWalkingStabilizationController_msgs.msg"
// #include "/*PackageName*//srv/ToKinematics_msgs.msg"

namespace walking_pattern_generator
{
  using namespace WalkingPatternGenerator;
  
  void callback_res(
    const rclcpp::Client</*PackageName*/::srv::ToKinematics_msgs>::SharedFuture future
  ) {
    // callback
  }

  WalkingPatternGenerator(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
  ) {
    // walking_pattern_generator
  }
}