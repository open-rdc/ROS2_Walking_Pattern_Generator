#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <rmw/qos_profiles.h>
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"

#include <chrono>

using namespace std::chrono_literals; 
using namespace std::placeholders;

namespace walking_pattern_generator
{
  // auto time = rclcpp::Clock{}.now().seconds();
  // auto time_max = time - time;
  // auto time_min = time + time;
  // int hoge = 0;

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
    // jointVelも、逆動力学（？）で導出したい。
    walking_pattern_jointVel_R_[0] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_R_[1] = {1, 1, 0.5, 1, 0.5, 1};
    walking_pattern_jointVel_R_[2] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_R_[3] = {1, 1, 0.5, 1, 0.5, 1};
    walking_pattern_jointVel_L_[0] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_L_[1] = {1, 1, 0.5, 1, 0.5, 1};
    walking_pattern_jointVel_L_[2] = {1, 1, 0.5, 1, 0.5, 1};  // [rad/s]
    walking_pattern_jointVel_L_[3] = {1, 1, 0.5, 1, 0.5, 1};

    loop_number_ = walking_pattern_P_R_.max_size();  // 要素の最大数を返す
  }

  void WalkingPatternGenerator::callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // resultをメンバ変数に記録。FK,IKそれぞれが求めない値（IK->p, FK->q）は、requestで与えた値と同値を返す。
    p_target_r_ = future.get()->p_result_r;
    p_target_l_ = future.get()->p_result_l;
    q_target_r_ = future.get()->q_result_r;
    q_target_l_ = future.get()->q_result_l;
    
    step_counter_++;
    publish_ok_check_ = true;

    // auto time2 = rclcpp::Clock{}.now().seconds();
    // if(hoge > 16){
    //   auto time_dev = time2 - time;
    //   if(time_max < time_dev){time_max = time_dev;}
    //   if(time_min > time_dev){time_min = time_dev;}
    //   std::cout << "[WalkingStabilizationController]: " << time_dev << "    max: " << time_max <<  "    min: " << time_min << std::endl;
    // }
    // hoge++;
    // time = time2;
  }

  void WalkingPatternGenerator::step_WPG_pub() {

    // RCLCPP_INFO(this->get_logger(), "step...");

    auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();
    auto toKine_IK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

    // DEBUG=====
    toKine_IK_req->r_target_r = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
    toKine_IK_req->p_target_r = walking_pattern_P_R_[step_counter_%loop_number_];  // [m]
    toKine_IK_req->r_target_l = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
    toKine_IK_req->p_target_l = walking_pattern_P_L_[step_counter_%loop_number_];  // [m]    
    // DEBUG=====

    // IK ERROR_Handling
    if(
      (std::abs(toKine_IK_req->p_target_r[2]) > 0.3082)  // コレだと不完全。absがある意味がない。他方向のERROR処理も随時追加
    ) {
      RCLCPP_ERROR(this->get_logger(), "IK_Request: P_target: Invalid Value!!");
    }

    // RCLCPP_INFO(this->get_logger(), "Request to kinematics...");
    auto toKine_IK_res = toKine_IK_clnt_->async_send_request(
      toKine_IK_req, 
      std::bind(&WalkingPatternGenerator::callback_res, this, _1)
    );

    auto pub_msg = std::make_shared<msgs_package::msg::ToWalkingStabilizationControllerMessage>();

    // set pub_msg
    pub_msg->p_target_r = p_target_r_;
    pub_msg->p_target_l = p_target_l_;
    pub_msg->q_target_r = q_target_r_;
    pub_msg->q_target_l = q_target_l_;
    pub_msg->dq_target_r = walking_pattern_jointVel_R_[(step_counter_-1)%loop_number_];
    pub_msg->dq_target_l = walking_pattern_jointVel_L_[(step_counter_-1)%loop_number_];

    if(publish_ok_check_ == true) {
      toWSC_pub_->publish(*pub_msg);
      // RCLCPP_INFO(this->get_logger(), "Published...");
    }
  }

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    // RCLCPP_INFO(this->get_logger(), "Start up WalkingPatternGenerator. Hello WalkingPatternGenerator!!");

    toKine_FK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>(
      "FK", 
      custom_qos_profile
    );
    toKine_IK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>(
      "IK",
      custom_qos_profile
    );

    toWSC_pub_ = this->create_publisher<msgs_package::msg::ToWalkingStabilizationControllerMessage>(
      "WalkingPattern",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile))
    );
    
    while(!toKine_IK_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: IK service is dead.");
        return;
      }
      // RCLCPP_INFO(this->get_logger(), "Waiting for IK service...");
    }

    // set inital counter value. set walking_pattern.
    publish_ok_check_ = false;
    step_counter_ = 0;

    // DEBUG: parameter setting
    WalkingPatternGenerator::DEBUG_ParameterSetting();

    step_pub_ = this->create_wall_timer(
      600ms,
      std::bind(&WalkingPatternGenerator::step_WPG_pub, this)
    );
  }
}