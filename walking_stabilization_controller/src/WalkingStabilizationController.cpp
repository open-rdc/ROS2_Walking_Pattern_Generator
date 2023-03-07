#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Msgs_Package/msg/ToWalkingStabilizationController_msgs.msg"
#include "Msgs_Package/srv/ToKinematics_msgs.msg"
#include "Msgs_Package/srv/ToWebotsRobotHandler_msgs.msg"
#include "Walking_Stabilization_Controller/WalkingStabilizationController.hpp"

namespace walking_stabilization_controller
{
  using namespace WalkingStabilizationController;

  void callback_sub(
    const Msgs_Package::msg::ToWalkingPatternGenerator_msgs::SharedPtr sub_data
  ) {
    // to walking_pattern_generator
  }

  void callback_res(
    const rclcpp::Client<Msgs_Package::srv::ToKinematics_msgs>::SharedFuture future
  ) {
    // to kinematics
  }

  void WSC_SrvServer(
    const std::shared_ptr<Msgs_Package::srv::ToWebotsRobotHandler_msgs::Request> request,
    std::shared_ptr<Msgs_Package::srv::ToWebotsRobotHandler_msgs::Response> response
  ) {
    // walking_stabilization_controller service_server
  }
}
