#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "msgs_package/msg/walking_pattern.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

using namespace Eigen;

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
    P_legL_ = {
        Vector3d(-0.005, 0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    P_legR_ = {
        Vector3d(-0.005, -0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };

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

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    // DEBUG: parameter setting
    WalkingPatternGenerator::DEBUG_ParameterSetting();

    using namespace std::chrono_literals;

    pub_walking_pattern_ = this->create_publisher<msgs_package::msg::WalkingPattern>("WalkingPattern", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile)));
    timer_ = create_wall_timer(500ms, std::bind(&WalkingPatternGenerator::WalkingPattern_Timer, this));
  }

  void WalkingPatternGenerator::WalkingPattern_Timer() {
    // RCLCPP_INFO(this->get_logger(), "WalkingPatternGenerator::WalkingPattern_Timer");
    // // stepの周期を元に、出力するwalking_patternを決定（今回は静歩行をループさせるので、/4して余りを算出）
    // step_count_ = request->step_count % 4;

    // // IdentifyMatrix（便利関数をまとめたライブラリで宣言・定義したい）
    // Eigen::Matrix3d I;
    // I << 1, 0, 0,
    //     0, 1, 0,
    //     0, 0, 1;

    // // response->leg_joint-angle
    // response->q_target_leg_r = IK_.getIK(
    //   P_legR_,                                        // leg_R joint point
    //   Vector3d(walking_pattern_P_R_[step_count_][0],  // walking_pattern axis_x
    //           walking_pattern_P_R_[step_count_][1],   // walking_pattern axis_y
    //           walking_pattern_P_R_[step_count_][2]),  // walking_pattern axis_z
    //   I                                               // foot angle matrix (IdentifyMatrix)
    // );
    // response->q_target_leg_l = IK_.getIK(
    //   P_legL_,                                        // leg_R joint point
    //   Vector3d(walking_pattern_P_L_[step_count_][0],  // walking_pattern axis_x
    //           walking_pattern_P_L_[step_count_][1],   // walking_pattern axis_y
    //           walking_pattern_P_L_[step_count_][2]),  // walking_pattern axis_z
    //   I                                               // foot angle matrix (IdentifyMatrix)
    // );

    // // response->leg_joint-velocity
    // response->dq_target_leg_r = walking_pattern_jointVel_R_[step_count_];
    // response->dq_target_leg_l = walking_pattern_jointVel_L_[step_count_];
  }
}