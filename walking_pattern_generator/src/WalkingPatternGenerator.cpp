#include "rclcpp/rclcpp.hpp"
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
    std::cout << "RESPONSE: \n" 
              << "__P_result_R: ";
    std::copy(std::begin(future.get()->p_result_r), 
              std::end(future.get()->p_result_r), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "__P_result_L: ";
    std::copy(std::begin(future.get()->p_result_l), 
              std::end(future.get()->p_result_l), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "__Q_result_R: ";
    std::copy(std::begin(future.get()->q_result_r), 
              std::end(future.get()->q_result_r), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "__Q_result_L: ";
    std::copy(std::begin(future.get()->q_result_l), 
              std::end(future.get()->q_result_l), 
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n" << std::endl;
    // DEBUG=====*/
  }

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    RCLCPP_INFO(this->get_logger(), "Start up WalkingPatternGenerator. Hello WalkingPatternGenerator!!");
    
    toKine_FK_clnt = this->create_client<msgs_package::srv::ToKinematicsMessage>("FK");

    while(!toKine_FK_clnt->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
    }

  auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

  // DEBUG=====/*
  // FK_request
  toKine_FK_req->q_target_r = {0, 0, 0, 0, 0, 0};
  toKine_FK_req->q_target_l = {0, 0, 0, 0, 0, 0};

  // IK_request
  toKine_FK_req->r_target_r = {0, 0, 0,
                               0, 0, 0,
                               0, 0, 0};
  toKine_FK_req->p_target_r = {0, 0, 0};
  toKine_FK_req->r_target_l = {0, 0, 0,
                               0, 0, 0,
                               0, 0, 0};
  toKine_FK_req->p_target_l = {0, 0, 0};
  // DEBUG=====*/

  auto toKine_FK_res = toKine_FK_clnt->async_send_request(
    toKine_FK_req, 
    std::bind(&WalkingPatternGenerator::callback_res, this, _1)
  );
  }
}