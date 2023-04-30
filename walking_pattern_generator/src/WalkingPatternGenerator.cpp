#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "msgs_package/srv/to_walking_pattern_generator.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

namespace walking_pattern_generator
{
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

  void WalkingPatternGenerator::DEBUG_ParameterSetting() {
    // 逆運動学からJointAngleを導出する。回転行列もWalkingPatternで欲しい？
    walking_pattern_P_R_[0] = {-0.01, -0.000, -0.3000};  // [m]
    walking_pattern_P_R_[1] = {-0.01, -0.000, -0.3000};
    walking_pattern_P_R_[2] = {0.01, -0.072, -0.2800};  // [m]
    walking_pattern_P_R_[3] = {0.01, -0.072, -0.2800};

    walking_pattern_P_L_[0] = {0.01, 0.072, -0.2800};  // [m]
    walking_pattern_P_L_[1] = {0.01, 0.072, -0.2800};
    walking_pattern_P_L_[2] = {-0.01, 0.000, -0.3000};  // [m]
    walking_pattern_P_L_[3] = {-0.01, 0.000, -0.3000};

    walking_pattern_jointVel_R_[0] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_R_[1] = {1, 1, 0.5, 1, 0.5, 1};
    walking_pattern_jointVel_R_[2] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_R_[3] = {1, 1, 0.5, 1, 0.5, 1};
    walking_pattern_jointVel_L_[0] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_L_[1] = {1, 1, 0.5, 1, 0.5, 1};
    walking_pattern_jointVel_L_[2] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_L_[3] = {1, 1, 0.5, 1, 0.5, 1};
  }

  void WalkingPatternGenerator::WPG_Server(
    const std::shared_ptr<msgs_package::srv::ToWalkingPatternGenerator::Request> resuest,
    std::shared_ptr<msgs_package::srv::ToWalkingPatternGenerator::Response> response
  ) {

  }

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    // DEBUG: parameter setting
    WalkingPatternGenerator::DEBUG_ParameterSetting();
  }
}