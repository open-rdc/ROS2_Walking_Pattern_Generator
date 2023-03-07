#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Msgs_Package/msg/ToWalkingStabilizationController_msgs.msg"
#include "Msgs_Package/srv/ToKinematics_msgs.msg"
#include "Walking_Pattern_Generator/WalkingPatternGenerator.hpp"

namespace walking_pattern_generator
{
  using namespace WalkingPatternGenerator;
  
  void callback_res(
    const rclcpp::Client<Msgs_Package::srv::ToKinematics_msgs>::SharedFuture future
  ) {
    // callback
  }

  WalkingPatternGenerator(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
  ) {
    // walking_pattern_generator
  }
}