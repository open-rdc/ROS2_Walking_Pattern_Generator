#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"

#include <chrono>

using namespace std::chrono_literals;  // 周期の単位を書けるようにする（ex. 100ms）
using namespace std::placeholders;  // bind()の第３引数etcを簡単にする（ex. _1）

namespace walking_pattern_generator
{
  void WalkingPatternGenerator::DEBUG_ParameterSetting() {
    // 逆運動学からJointAngleを導出する。回転行列もWalkingPatternで欲しい？
    walking_pattern_P_R_[0] = {-0.005, -0.000, -0.3000};  // [m]
    walking_pattern_P_R_[1] = {-0.005, -0.000, -0.3000};
    walking_pattern_P_R_[2] = {-0.005, -0.072, -0.2800};  // [m]
    walking_pattern_P_R_[3] = {-0.005, -0.072, -0.2800};

    walking_pattern_P_L_[0] = {-0.005, 0.072, -0.2800};  // [m]
    walking_pattern_P_L_[1] = {-0.005, 0.072, -0.2800};
    walking_pattern_P_L_[2] = {-0.005, 0.000, -0.3000};  // [m]
    walking_pattern_P_L_[3] = {-0.005, 0.000, -0.3000};
    // jointVelも、逆動力学（？）で導出したい。
    walking_pattern_jointVel_R_[0] = {2, 2, 1, 2, 1, 2};  // [rad/s]
    walking_pattern_jointVel_R_[1] = {2, 2, 1, 2, 1, 2};
    walking_pattern_jointVel_R_[2] = {2, 2, 1, 2, 1, 2};  // [rad/s]
    walking_pattern_jointVel_R_[3] = {2, 2, 1, 2, 1, 2};
    walking_pattern_jointVel_L_[0] = {2, 2, 1, 2, 1, 2};  // [rad/s]
    walking_pattern_jointVel_L_[1] = {2, 2, 1, 2, 1, 2};
    walking_pattern_jointVel_L_[2] = {2, 2, 1, 2, 1, 2};  // [rad/s]
    walking_pattern_jointVel_L_[3] = {2, 2, 1, 2, 1, 2};

    loop_number_ = walking_pattern_P_R_.max_size();  // 要素の最大数を返す
    // while(rclcpp::ok()){
    // std::cout << loop_number_ << std::endl;
    // }
  }

  void WalkingPatternGenerator::callback_res(
    const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  ) {
    // DEBUG=====/*
    RCLCPP_INFO(this->get_logger(), "RESPONSE: ");
    std::cout << "__P_result_R: ";
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

    // resultをメンバ変数に記録。FK,IKそれぞれが求めない値（IK->p, FK->q）は、requestで与えた値と同値を返す。
    p_target_r_ = future.get()->p_result_r;
    p_target_l_ = future.get()->p_result_l;
    q_target_r_ = future.get()->q_result_r;
    q_target_l_ = future.get()->q_result_l;
    
    step_counter_++;
    publish_ok_check_ = true;
  }

  void WalkingPatternGenerator::step_WPG_pub() {

    RCLCPP_INFO(this->get_logger(), "step...");

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
    toKine_IK_req->p_target_l = walking_pattern_P_L_[step_counter_%loop_number_
    ];  // [m]    
    // DEBUG=====

    // FK ERROR_Handling
    for(int i = 0; i < (int)toKine_FK_req->p_target_r.size(); i++) {
      if(
        (std::abs(toKine_FK_req->q_target_r[i]) > 3.14) or
        (std::abs(toKine_FK_req->q_target_l[i]) > 3.14)
      ) { 
        RCLCPP_ERROR(this->get_logger(), "FK_Request: Q_Target: Invalid Value!!");
        return;
      }
    }
    // IK ERROR_Handling
    if(
      (std::abs(toKine_IK_req->p_target_r[2]) > 0.3082)  // コレだと不完全。absがある意味がない。他方向のERROR処理も随時追加
    ) {
      RCLCPP_ERROR(this->get_logger(), "IK_Request: P_target: Invalid Value!!");
    }

    // 非同期の待ち状態。待ちつつも、以降のプログラムを実行。このまま（2023/4/1/17:06）だと、responseを受ける前にpublishしてしまう。= resを受け取るより先に以降のプログラムが実行済みになってしまう。ここは、responseを待つ、同期処理にすべき、なのだが、spin_until_future_complete()の引数でthis->...の箇所で、std::runtime_errorを吐いてくる。無理。responseのほうがpublishよりも、約2.4[ms]遅い。

    // auto toKine_FK_res = toKine_FK_clnt_->async_send_request(
    //   toKine_FK_req, 
    //   std::bind(&WalkingPatternGenerator::callback_res, this, _1)
    // );
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
      RCLCPP_INFO(this->get_logger(), "Publish...");
    }
  }

  WalkingPatternGenerator::WalkingPatternGenerator(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingPatternGenerator", options) {

    RCLCPP_INFO(this->get_logger(), "Start up WalkingPatternGenerator. Hello WalkingPatternGenerator!!");

    toKine_FK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("FK");
    toKine_IK_clnt_ = this->create_client<msgs_package::srv::ToKinematicsMessage>("IK");

    toWSC_pub_ = this->create_publisher<msgs_package::msg::ToWalkingStabilizationControllerMessage>("WalkingPattern", rclcpp::QoS(10));
    
    // while(!toKine_FK_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
    //     return;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
    // }
    // while(!toKine_IK_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR!!: IK service is dead.");
    //     return;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "Waiting for IK service...");
    // }

    // set inital counter value. set walking_pattern.
    publish_ok_check_ = false;
    step_counter_ = 0;

    // DEBUG: parameter setting
    WalkingPatternGenerator::DEBUG_ParameterSetting();
    
    // Timer処理。指定の周期で指定の関数を実行
    step_pub_ = this->create_wall_timer(
      600ms,
      std::bind(&WalkingPatternGenerator::step_WPG_pub, this)
    );
  }
}