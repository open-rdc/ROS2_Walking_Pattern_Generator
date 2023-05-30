#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "robot_manager/robot_manager.hpp"
#include "msgs_package/msg/control_output.hpp"

// using namespace std::placeholders;

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

  RobotManager::RobotManager(
    const rclcpp::NodeOptions &options
  ) : Node("RobotManager", options) {

    using namespace std::chrono_literals;

    clnt_stabilization_control_ = this->create_client<msgs_package::srv::StabilizationControl>(
      "StabilizationControl",
      custom_qos_profile
    );
    while(!clnt_stabilization_control_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: StabilizationControl service is dead.");
        return;
      }
    }

    pub_control_output_ = this->create_publisher<msgs_package::msg::ControlOutput>("ControlOutput", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile)));
    timer_ = create_wall_timer(10ms, std::bind(&RobotManager::ControlOutput_Timer, this));
  }

  // Robot Manager 
  void RobotManager::ControlOutput_Timer() {
    // RCLCPP_INFO(this->get_logger(), "RobotManager");
  }

  void RobotManager::WalkingPattern_Callback(const msgs_package::msg::WalkingPattern::SharedPtr callback_data) {
    // RCLCPP_INFO(this->get_logger(), "RobotManager::WalkingPattern_Callback");
  }

  void RobotManager::Feedback_Callback(const msgs_package::msg::Feedback::SharedPtr callback_data) {
    // RCLCPP_INFO(this->get_logger(), "RobotManager::Feedback");
  }
}