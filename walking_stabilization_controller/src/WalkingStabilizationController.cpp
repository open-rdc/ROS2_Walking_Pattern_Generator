#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"
#include "walking_stabilization_controller/WalkingStabilizationController.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace walking_stabilization_controller
{
  void WalkingStabilizationController::callback_sub(
    const msgs_package::msg::ToWalkingStabilizationControllerMessage::SharedPtr sub_data
  ) {
    // to walking_pattern_generator
  }

  void WalkingStabilizationController::callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // to kinematics
  }

  void WalkingStabilizationController::WSC_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Response> response
  ) {
    // walking_stabilization_controller service_server
  }

  WalkingStabilizationController::WalkingStabilizationController(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingStabilizationController", options) {

    RCLCPP_INFO(this->get_logger(), "Start up WalkingStabilizationController. Hello WalkingStabilizationController!!");

    toKine_FK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("FK");
    toKine_IK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("IK");

    // while(!toKine_FK_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
    //     return;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
    // }
    // while(!toKine_IK_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR!!: IK service is dead.");
    //     return;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "Waiting for IK service...");
    // }

    toWSC_sub_ = this->create_subscription<msgs_package::msg::ToWalkingStabilizationControllerMessage>("WalkingPattern", 
         rclcpp::QoS(10), 
         std::bind(&WalkingStabilizationController::callback_sub, this, _1));
    toWRH_srv_ = this->create_service<msgs_package::srv::ToWebotsRobotHandlerMessage>(
      "FB_StabilizationController",
      std::bind(&WalkingStabilizationController::WSC_SrvServer, this, _1, _2)
    );

    RCLCPP_INFO(this->get_logger(), "Waiting request & publish ...");
  }
}
