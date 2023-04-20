#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <rmw/qos_profiles.h>
#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "Eigen/Dense"

#include <chrono>

using namespace std::chrono_literals; 
using namespace std::placeholders;
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


  double Tc;  // 時定数（sqrt(com_z/g)）
  double C;  // cosh(t_sup_/Tc)
  double S;  // sinh(t_sup_/Tc)

  void WalkingPatternGenerator::DEBUG_ParameterSetting() {
    UnitVec_legR_ = {  // legR joint unit vector
      Vector3d(0, 0, 1),
      Vector3d(1, 0, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(1, 0, 0)
    };
    UnitVec_legL_ = {  // legL joint unit vector
      Vector3d(0, 0, 1),
      Vector3d(1, 0, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(1, 0, 0)      
    };

      // 初期重心位置(x, y)
      // Q(0, 0, -0.322497(-3.14/8), 0.784994(3.14/4), -0.392497(-3.14/8), 0)
      // legR_y: -0.037, legL_y: 0.037, 間をy軸として、com_y: 0.0
      walking_pattern_init_com_ << -0.005, 0.0;  
      init_com_z_ = -0.294056;  // 必要？いらないはず。 一応、記録用。

      // 歩行パラメータ(x1~5, y1~5)
      walking_pattern_s_ << 0.000, 0.100, 0.100, 0.100, 0.000,  // x
                            0.072, 0.072, 0.072, 0.072, 0.072   // y
                        ;

      t_sup_ = 600;
      Tc = sqrt(init_com_z_ / 9.80665);
      C = std::cosh(t_sup_/Tc);
      S = std::sinh(t_sup_/Tc);

    // loop_number_ = walking_pattern_P_R_.max_size();  // 要素の最大数を返す
  }


  void WalkingPatternGenerator::JacobiMatrix_leg(std::array<double, 6> Q_legR, std::array<double, 6> Q_legL) {
    Jacobi_legR_ = MatrixXd::Zero(6, UnitVec_legR_.max_size());
    Jacobi_legL_ = MatrixXd::Zero(6, UnitVec_legR_.max_size());

    auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

    toKine_FK_req->q_target_r = Q_legR;
    toKine_FK_req->q_target_l = Q_legL;

    for(int i = 0; i < int(UnitVec_legR_.max_size()); i++) {
      toKine_FK_req->fk_point = i;

      auto toKine_FK_res = toKine_FK_clnt_->async_send_request(
        toKine_FK_req, 
        [this, i](const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future) {
          P_FK_legR_[i] = {future.get()->p_result_r[0], future.get()->p_result_r[1], future.get()->p_result_r[2]};
          P_FK_legL_[i] = {future.get()->p_result_l[0], future.get()->p_result_l[1], future.get()->p_result_l[2]};
        }
      );
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), toKine_FK_res);

      // std::cout << i << std::endl;
      // std::cout << "legR: " << P_FK_legR_[i].transpose() << std::endl;
      // std::cout << "legL: " << P_FK_legL_[i].transpose() << std::endl;
    }
    std::cout << std::endl;

    Vector3d P_legR = P_FK_legR_[int(UnitVec_legR_.max_size())-1];
    Vector3d P_legL = P_FK_legL_[int(UnitVec_legR_.max_size())-1];
    

    Vector3d mat_legR = Vector3d::Zero(3);
    Vector3d mat_legL = Vector3d::Zero(3);
    Vector3d pt_P_legR = Vector3d::Zero(3);
    Vector3d pt_P_legL = Vector3d::Zero(3);
    for(int tag = 0; tag < int(UnitVec_legR_.max_size()); tag++) {
      if(tag == int(UnitVec_legR_.max_size()-1)) {
        mat_legR = Vector3d::Zero(3);
        mat_legL = Vector3d::Zero(3);
      }
      else { 
        pt_P_legR = P_legR - P_FK_legR_[tag];
        pt_P_legL = P_legL - P_FK_legL_[tag];
        // std::cout << "pt_P_legR: " << pt_P_legR.transpose() << std::endl;
        // std::cout << "pt_P_legL: " << pt_P_legL.transpose() << std::endl;
        mat_legR = UnitVec_legR_[tag].cross(pt_P_legR);
        mat_legL = UnitVec_legL_[tag].cross(pt_P_legL);
      }

      for(int i = 0; i < 3; i++) {
        if(abs(mat_legR[i]) < 0.000001) {
          mat_legR[i] = 0;
        }
        if(abs(mat_legL[i]) < 0.000001) {
          mat_legL[i] = 0;
        }
      }

      for(int i = 0; i < 3; i++) {
        Jacobi_legR_(i, tag) = mat_legR[i];
        Jacobi_legR_(i+3, tag) = UnitVec_legR_[tag][i];
        Jacobi_legL_(i, tag) = mat_legL[i];
        Jacobi_legL_(i+3, tag) = UnitVec_legL_[tag][i];
      }
    }
  }

  std::array<double, 6> WalkingPatternGenerator::Vector2Array(Vector<double, 6> vector) {
    return (std::array<double, 6>{vector[0], vector[1], vector[2], vector[3], vector[4], vector[5]});
  }


  // void WalkingPatternGenerator::callback_IK_res(
  //   const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future
  // ) {
  //   // resultをメンバ変数に記録。FK,IKそれぞれが求めない値（IK->p, FK->q）は、requestで与えた値と同値を返す。
  //   p_target_r_ = future.get()->p_result_r;
  //   p_target_l_ = future.get()->p_result_l;
  //   q_target_r_ = future.get()->q_result_r;
  //   q_target_l_ = future.get()->q_result_l;
    
  //   // step_counter_++;
  //   publish_ok_check_ = true;
  // }

  void WalkingPatternGenerator::step_WPG_pub() {

    // RCLCPP_INFO(this->get_logger(), "step...");

    auto toKine_IK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

    // IK ERROR_Handling
    if(
      (std::abs(toKine_IK_req->p_target_r[2]) > 0.3082)  // コレだと不完全。absがある意味がない。他方向のERROR処理も随時追加
    ) {
      RCLCPP_ERROR(this->get_logger(), "IK_Request: P_target: Invalid Value!!");
    }

    // RCLCPP_INFO(this->get_logger(), "Request to kinematics...");
    auto toKine_IK_res = toKine_IK_clnt_->async_send_request(
      toKine_IK_req, 
      // std::bind(&WalkingPatternGenerator::callback_IK_res, this, _1)
      [this](const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future) {

        // resultをメンバ変数に記録。FK,IKそれぞれが求めない値（IK->p, FK->q）は、requestで与えた値と同値を返す。
        p_target_r_ = future.get()->p_result_r;
        p_target_l_ = future.get()->p_result_l;
        q_target_r_ = future.get()->q_result_r;
        q_target_l_ = future.get()->q_result_l;
        
        // step_counter_++;
        publish_ok_check_ = true;
      }
    );
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), toKine_IK_res);

    // // get JacobiMatrix
    // WalkingPatternGenerator::JacobiMatrix_leg(q_target_r_, q_target_l_);

    // // DEBUG
    // Vector3d v_legR = {0, 0, 0.1, 0, 0, 0};  // 脚の末端の欲しい速度・角速度（符号を逆にしてやれば、基準点（＝胴体）の欲しい速度・角速度（な、はず））
    // Vector3d v_legL = {0, 0, 1.0, 0, 0, 0};  // 遊脚は、符号が基準と同じ。支持脚は、符号が基準と逆。

    // auto dq_target_r = Vector2Array(Jacobi_legR_.inverse() * v_legR);  // 各関節角速度を計算
    // auto dq_target_l = Vector2Array(Jacobi_legL_.inverse() * v_legL);

    // auto pub_msg = std::make_shared<msgs_package::msg::ToWalkingStabilizationControllerMessage>();

    // // set pub_msg
    // pub_msg->p_target_r = p_target_r_;
    // pub_msg->p_target_l = p_target_l_;
    // pub_msg->q_target_r = q_target_r_;
    // pub_msg->q_target_l = q_target_l_;
    // pub_msg->dq_target_r = dq_target_r;
    // pub_msg->dq_target_l = dq_target_l;

    // if(publish_ok_check_ == true) {
    //   toWSC_pub_->publish(*pub_msg);
    //   // RCLCPP_INFO(this->get_logger(), "Published...");
    // }
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

    
    while(!toKine_FK_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
        return;
      }
      // RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
    }    
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
      std::chrono::milliseconds(t_sup_),  // t_sup_[ms]
      std::bind(&WalkingPatternGenerator::step_WPG_pub, this)
    );
  }
}