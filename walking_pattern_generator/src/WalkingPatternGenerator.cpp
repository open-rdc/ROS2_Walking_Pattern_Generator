#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace walking_pattern_generator
{
  void WalkingPatternGenerator::callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // DEBUG=====/*
    std::cout << "RESPONSE" 
              << "\n" << "__P_result_R: ";
    std::copy(std::begin(future.get()->p_result_r), 
              std::end(future.get()->p_result_r), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n" << "__P_result_L: ";
    std::copy(std::begin(future.get()->p_result_l), 
              std::end(future.get()->p_result_l), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n" << "__Q_result_R: ";
    std::copy(std::begin(future.get()->q_result_r), 
              std::end(future.get()->q_result_r), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n" << "__Q_result_L: ";
    std::copy(std::begin(future.get()->q_result_l), 
              std::end(future.get()->q_result_l), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n" << std::endl;

    // resultをメンバ変数に記録。FK,IKそれぞれが求めない値（IK->p, FK->q）は、requestで与えた値と同値を返す。
    p_target_r_ = future.get()->p_result_r;
    p_target_l_ = future.get()->p_result_l;
    q_target_r_ = future.get()->q_result_r;
    q_target_l_ = future.get()->q_result_l;
    // DEBUG=====*/
  }

  void WalkingPatternGenerator::step_WPG_pub() {

    RCLCPP_INFO(this->get_logger(), "step...");

    auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();
    auto toKine_IK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

    // DEBUG=====
    // FK_request
    toKine_FK_req->q_target_r = {0, 0, 0, 0, 0, 0};  // [rad]
    toKine_FK_req->q_target_l = {0, 0, 0, 0, 0, 0};  // [rad]

    // IK_request
    toKine_IK_req->r_target_r = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
    toKine_IK_req->p_target_r = {0, 0, 0};  // [m]
    toKine_IK_req->r_target_l = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
    toKine_IK_req->p_target_l = {0, 0, 0};  // [m]
    // DEBUG=====

    auto toKine_FK_res = toKine_FK_clnt_->async_send_request(
      toKine_FK_req, 
      std::bind(&WalkingPatternGenerator::callback_res, this, _1)
    );

    auto pub_msg = std::make_shared<msgs_package::msg::ToWalkingStabilizationControllerMessage>();

    // pub_msg->p_target_r = {1, 1, 1};
    // pub_msg->p_target_l = {2, 2, 2};
    pub_msg->p_target_r = p_target_r_;
    pub_msg->p_target_l = p_target_l_;
    // pub_msg->q_target_r = {3, 3, 3, 3, 3, 3};
    // pub_msg->q_target_l = {4, 4, 4, 4, 4, 4};
    pub_msg->q_target_r = q_target_r_;
    pub_msg->q_target_l = q_target_l_;
    pub_msg->dq_target_r = {5, 5, 5, 5, 5, 5};
    pub_msg->dq_target_l = {6, 6, 6, 6, 6, 6};

    toWSC_pub_->publish(*pub_msg);
    RCLCPP_INFO(this->get_logger(), "Publish...");
  }

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    RCLCPP_INFO(this->get_logger(), "Start up WalkingPatternGenerator. Hello WalkingPatternGenerator!!");
    
    toKine_FK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("FK");
    toKine_IK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("IK");

    toWSC_pub_ = this->create_publisher<msgs_package::msg::ToWalkingStabilizationControllerMessage>("WalkingPattern", rclcpp::QoS(10));
    

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


    step_pub_ = this->create_wall_timer(
      1000ms,
      std::bind(&WalkingPatternGenerator::step_WPG_pub, this)
    );
  }
}