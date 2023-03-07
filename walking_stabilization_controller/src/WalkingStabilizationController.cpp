#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"
#include "Walking_Stabilization_Controller/WalkingStabilizationController.hpp"

namespace walking_stabilization_controller
{
  using namespace WalkingStabilizationController;

  void callback_sub(
    const msgs_package::msg::ToWalkingStabilizationControllerMessage::SharedPtr sub_data
  ) {
    // to walking_pattern_generator
  }

  void callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // to kinematics
  }

  void WSC_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Response> response
  ) {
    // walking_stabilization_controller service_server
  }
}
