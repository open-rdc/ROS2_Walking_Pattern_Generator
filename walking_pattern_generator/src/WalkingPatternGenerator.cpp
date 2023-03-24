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
    // DEBUG=====*/
  }

  void WalkingPatternGenerator::step_WPG_pub() {
/*
    auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();
    auto toKine_IK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

    // DEBUG=====
    // IK_request
    toKine_IK_req->q_target_r = {0, 0, 0, 0, 0, 0};  // [rad]
    toKine_IK_req->q_target_l = {0, 0, 0, 0, 0, 0};  // [rad]

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

    auto toKine_IK_res = toKine_IK_clnt->async_send_request(
      toKine_IK_req, 
      std::bind(&WalkingPatternGenerator::callback_res, this, _1)
    );
*/
    auto pub_msg = std::make_shared<msgs_package::msg::ToWalkingStabilizationControllerMessage>();

    pub_msg->p_target_r = {0, 0, 0};
    pub_msg->p_target_l = {0, 0, 0};
    pub_msg->q_target_r = {0, 0, 0, 0, 0, 0};
    pub_msg->q_target_l = {0, 0, 0, 0, 0, 0};
    pub_msg->dq_target_r = {0, 0, 0, 0, 0, 0};
    pub_msg->dq_target_l = {0, 0, 0, 0, 0, 0};

    toWSC_pub_->publish(*pub_msg);
  }

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    RCLCPP_INFO(this->get_logger(), "Start up WalkingPatternGenerator. Hello WalkingPatternGenerator!!");
    
    toKine_FK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("FK");
    toKine_IK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("IK");

    toWSC_pub_ = this->create_publisher<msgs_package::msg::ToWalkingStabilizationControllerMessage>("WalkingPattern", rclcpp::QoS(10));
    
/*
    while(!toKine_FK_clnt->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
    }
    while(!toKine_IK_clnt->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: IK service is dead.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for IK service...");
    }
*/

    step_pub_ = this->create_wall_timer(
      100ms,
      std::bind(&WalkingPatternGenerator::step_WPG_pub, this)
    );
  }
}