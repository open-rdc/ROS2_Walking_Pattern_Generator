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

  // Robot Manager Service server
  void RobotManager::RM_Server(
    const std::shared_ptr<msgs_package::srv::ToRobotManager::Request> request,
    std::shared_ptr<msgs_package::srv::ToRobotManager::Response> response
  ) {
    RCLCPP_INFO(get_logger(), "hoge");
    // auto WPG_clnt_req = std::make_shared<msgs_package::srv::ToWalkingPatternGenerator::Request>();
    // auto WSC_clnt_req = std::make_shared<msgs_package::srv::ToWalkingStabilizationController::Request>();

    // // set WPG_req
    // WPG_clnt_req->step_count = request->step_count;

    // // req WPG & wait WPG_res
    // auto WPG_future = WPG_clnt_->async_send_request(WPG_clnt_req);
    // // WPG_future.wait();
    // // std::future_status future_status = WPG_future.wait_for(300ms);  // wait for 3ms. 3ms or future_ready
    // // if(future_status == std::future_status::ready) {  // get WPG_res & set WSC_req
    //   WSC_clnt_req->q_now_leg_r = request->q_now_leg_r;
    //   WSC_clnt_req->q_now_leg_l = request->q_now_leg_l;
    //   WSC_clnt_req->q_target_leg_r = WPG_future.get()->q_target_leg_r;  // future.get()でも、responseをwaitできる。
    //   WSC_clnt_req->q_target_leg_l = WPG_future.get()->q_target_leg_l;
    //   WSC_clnt_req->dq_target_leg_r = WPG_future.get()->dq_target_leg_r;
    //   WSC_clnt_req->dq_target_leg_l = WPG_future.get()->dq_target_leg_l;
    //   WSC_clnt_req->accelerometer_now = request->accelerometer_now;
    //   WSC_clnt_req->gyro_now = request->gyro_now;    
    // // }
    // // else if(future_status == std::future_status::timeout) {  // failed
    // //   RCLCPP_WARN(this->get_logger(), "<TIMEOUT> WPG_future: Time over 3ms");
    // // }

    // // req WSC & wait WSC_res
    // auto WSC_future = WSC_clnt_->async_send_request(WSC_clnt_req);
    // // WSC_future.wait();
    // // future_status = WSC_future.wait_for(300ms);
    // // if(future_status == std::future_status::ready) {  // get WSC_res & set RM_res
    //   response->q_next_leg_r = WSC_future.get()->q_next_leg_r;
    //   response->q_next_leg_l = WSC_future.get()->q_next_leg_l;
    //   response->dq_next_leg_r = WSC_future.get()->dq_next_leg_r;
    //   response->dq_next_leg_l = WSC_future.get()->dq_next_leg_l;
    // // }
    // // else if(future_status == std::future_status::timeout) {  // failed
    // //   RCLCPP_WARN(this->get_logger(), "<TIMEOUT> WSC_future: Time over 3ms");
    // // }
    
  }

  RobotManager::RobotManager(
    const rclcpp::NodeOptions &options
  ) : Node("RobotManager", options) {
    
    // WPG_clnt_ = this->create_client<msgs_package::srv::ToWalkingPatternGenerator>(
    //   "WalkingPattern",
    //   custom_qos_profile,
    //   callback_group_
    // );
    // WSC_clnt_ = this->create_client<msgs_package::srv::ToWalkingStabilizationController>(
    //   "StabilizationControl",
    //   custom_qos_profile,
    //   callback_group_
    // );

    // while(!WPG_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR!!: WalkingPattern service is dead.");
    //     return;
    //   }
    // }
    // while(!WSC_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR!!: StabilizationControl service is dead.");
    //     return;
    //   }
    // }

    RM_srv_ = this->create_service<msgs_package::srv::ToRobotManager>(
      "RobotManage",
      std::bind(&RobotManager::RM_Server, this, _1, _2),
      custom_qos_profile
      // callback_group_
    );
  }
}