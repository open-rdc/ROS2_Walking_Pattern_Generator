#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "/*PackageName*//msg/ToWalkingStabilizationController_msgs.msg"
// #include "/*PackageName*//srv/ToKinematics_msgs.msg"
// #include "/*PackageName*//srv/ToWebotsRobotHandler_msgs.msg"

namespace walking_stabilization_controller
{
  using namespace WalkingStabilizationController;

  void callback_sub(
    const /*PackageName*/::msg::ToWalkingPatternGenerator_msgs::SharedPtr sub_data
  ) {
    // to walking_pattern_generator
  }

  void callback_res(
    const rclcpp::Client</*PackageName*/::srv::ToKinematics_msgs>::SharedFuture future
  ) {
    // to kinematics
  }

  void WSC_SrvServer(
    const std::shared_ptr</*PackageName*/::srv::ToWebotsRobotHandler_msgs::Request> request,
    std::shared_ptr</*PackageName*/::srv::ToWebotsRobotHandler_msgs::Response> response
  ) {
    // walking_stabilization_controller service_server
  }
}
