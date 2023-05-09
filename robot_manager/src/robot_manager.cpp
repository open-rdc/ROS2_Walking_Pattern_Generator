#include "rclcpp/rclcpp.hpp"
#include "robot_manager/robot_manager.hpp"
#include "msgs_package/srv/to_robot_manager.hpp"
#include "msgs_package/srv/to_walking_pattern_generator.hpp"
#include "msgs_package/srv/to_walking_stabilization_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robot_manager {
  static const rmw_qos_profile_t custom_qos_profile =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
    1,  // History(keep_last) Depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
    RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
    RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
    RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
    false  // avoid_ros_namespace_conventions
  };

  void RobotManager::RM_Server(
    const std::shared_ptr<msgs_package::srv::ToRobotManager::Request> requset,
    std::shared_ptr<msgs_package::srv::ToRobotManager::Response> response
  ) {

  }

  RobotManager::RobotManager(
    const rclcpp::NodeOptions &options
  ) : Node("RobotManager", options) {
    
    WPG_clnt_ = this->create_client<msgs_package::srv::ToWalkingPatternGenerator>(
      "WalkingPattern",
      custom_qos_profile
    );
    WSC_clnt_ = this->create_client<msgs_package::srv::ToWalkingStabilizationController>(
      "StabilizationControl",
      custom_qos_profile
    );

    while(!WPG_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: WalkingPattern service is dead.");
        return;
      }
    }
    while(!WSC_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: StabilizationControl service is dead.");
        return;
      }
    }

    RM_srv_ = this->create_service<msgs_package::srv::ToRobotManager>(
      "RobotManage",
      std::bind(&RobotManager::RM_Server, this, _1, _2),
      custom_qos_profile
    );
  }
}