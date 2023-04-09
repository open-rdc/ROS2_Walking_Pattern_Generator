#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <rmw/qos_profiles.h>
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"
#include "walking_stabilization_controller/WalkingStabilizationController.hpp"

#include <chrono>
#include "iostream"
#include "cmath"
#include "Eigen/Dense"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace walking_stabilization_controller
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

  void WalkingStabilizationController::callback_sub(
    const msgs_package::msg::ToWalkingStabilizationControllerMessage::SharedPtr sub_data
  ) {
    // to walking_pattern_generator

    RCLCPP_INFO(this->get_logger(), "Start callback_sub");

    P_target_legR_ = {sub_data->p_target_r[0], sub_data->p_target_r[1], sub_data->p_target_r[2]};
    P_target_legL_ = {sub_data->p_target_l[0], sub_data->p_target_l[1], sub_data->p_target_l[2]};
    Q_target_legR_ = sub_data->q_target_r;
    Q_target_legL_ = sub_data->q_target_l;
    dQ_target_legR_ = sub_data->dq_target_r;
    dQ_target_legL_ = sub_data->dq_target_l;

    std::cout << P_target_legR_.transpose() << std::endl;
    std::cout << P_target_legL_.transpose() << std::endl;
    for(int i = 0; i < 6; i++){std::cout << Q_target_legR_[i];} std::cout << std::endl;
    for(int i = 0; i < 6; i++){std::cout << Q_target_legL_[i];} std::cout << std::endl;
    for(int i = 0; i < 6; i++){std::cout << dQ_target_legR_[i];} std::cout << std::endl;
    for(int i = 0; i < 6; i++){std::cout << dQ_target_legL_[i];} std::cout << std::endl;

    RCLCPP_INFO(this->get_logger(), "Finish callback_sub\n");
  }

  void WalkingStabilizationController::callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // to kinematics

    RCLCPP_INFO(this->get_logger(), "Start callback_res");

    // FK, IKのresultをメンバ変数に記録。FK,IKそれぞれが求めない値（IK->p, FK->q）は、requestで与えた値と同値を返す。
    P_target_legR_ = {future.get()->p_result_r[0], future.get()->p_result_r[1], future.get()->p_result_r[2]};
    P_target_legL_ = {future.get()->p_result_l[0], future.get()->p_result_l[1], future.get()->p_result_l[2]};
    Q_target_legR_ = {future.get()->q_result_r[0], future.get()->q_result_r[1], future.get()->q_result_r[2], future.get()->q_result_r[3], future.get()->q_result_r[4], future.get()->q_result_r[5]};
    Q_target_legL_ = {future.get()->q_result_l[0], future.get()->q_result_l[1], future.get()->q_result_l[2], future.get()->q_result_l[3], future.get()->q_result_l[4], future.get()->q_result_l[5]};

    Q_fix_legR_ = Q_target_legR_;
    Q_fix_legL_ = Q_target_legL_;
    dQ_fix_legR_ = dQ_target_legR_;
    dQ_fix_legL_ = dQ_target_legL_;

    RCLCPP_INFO(this->get_logger(), "Finish callback_res");
    return;
  }

  void WalkingStabilizationController::WSC_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Response> response
  ) {
    // walking_stabilization_controller service_server

    RCLCPP_INFO(this->get_logger(), "Start WSC_SrvServer");

    if(P_target_legR_[0] == 999) {
      RCLCPP_WARN(this->get_logger(), "WSC couldn't get subscription from WPG. All value are numeric 999.");

      response->q_fix_r = Q_target_legR_;
      response->q_fix_l = Q_target_legL_;
      response->dq_fix_r = dQ_target_legR_;
      response->dq_fix_l = dQ_target_legL_;

      RCLCPP_INFO(this->get_logger(), "Finish WSC_SrvServer");
      return;
    }

    auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage_Request>();
    auto toKine_IK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage_Request>();

    toKine_FK_req->q_target_r = Q_target_legR_;
    toKine_FK_req->q_target_l = Q_target_legL_;

    toKine_IK_req->p_target_r = {P_target_legR_[0], P_target_legR_[1], P_target_legR_[2]};
    toKine_IK_req->p_target_l = {P_target_legL_[0], P_target_legL_[1], P_target_legL_[2]};
    toKine_IK_req->r_target_r = {1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1};
    toKine_IK_req->r_target_l = {1, 0, 0, 
                                 0, 1, 0,
                                 0, 0, 1};

    auto toKine_FK_res = toKine_FK_clnt_->async_send_request(
      toKine_FK_req, 
      std::bind(&WalkingStabilizationController::callback_res, this, _1)
    );
    // auto toKine_IK_res = toKine_IK_clnt_->async_send_request(
    //   toKine_IK_req,
    //   std::bind(&WalkingStabilizationController::callback_res, this, _1)
    // );

    response->q_fix_r = Q_fix_legR_;
    response->q_fix_l = Q_fix_legL_;
    response->dq_fix_r = dQ_fix_legR_;
    response->dq_fix_l = dQ_fix_legL_;

    RCLCPP_INFO(this->get_logger(), "Finish WSC_SrvServer");
  }

  WalkingStabilizationController::WalkingStabilizationController(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingStabilizationController", options) {

    RCLCPP_INFO(this->get_logger(), "Start up WalkingStabilizationController. Hello WalkingStabilizationController!!");

    toKine_FK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>(
      "FK",
      custom_qos_profile
    );
    toKine_IK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>(
      "IK",
      custom_qos_profile
    );

    while(!toKine_FK_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
    }
    while(!toKine_IK_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: IK service is dead.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for IK service...");
    }

    toWSC_sub_ = this->create_subscription<msgs_package::msg::ToWalkingStabilizationControllerMessage>(
      "WalkingPattern", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile)), 
      std::bind(&WalkingStabilizationController::callback_sub, this, _1));
    toWRH_srv_ = this->create_service<msgs_package::srv::ToWebotsRobotHandlerMessage>(
      "FB_StabilizationController",
      std::bind(&WalkingStabilizationController::WSC_SrvServer, this, _1, _2),
      custom_qos_profile
    );

    // init
    P_target_legR_ = {999, 999, 999};
    P_target_legL_ = {999, 999, 999};
    Q_target_legR_ = {999, 999, 999, 999, 999, 999};
    Q_target_legL_ = {999, 999, 999, 999, 999, 999};
    dQ_target_legR_ = {999, 999, 999, 999, 999, 999};
    dQ_target_legL_ = {999, 999, 999, 999, 999, 999};;

    RCLCPP_INFO(this->get_logger(), "Waiting request & publish ...");
  }
}
