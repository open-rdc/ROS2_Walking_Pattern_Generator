#include "rclcpp/rclcpp.hpp"
#include <rmw/types.h>
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "kinematics/FK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace kinematics
{
  // auto time = rclcpp::Clock{}.now().seconds();
  // auto time_max = time - time;
  // auto time_min = time + time;

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

  using namespace Eigen;

  // 3D Rotation Matrix
  Matrix3d FKSrv::IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d FKSrv::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d FKSrv::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d FKSrv::Rz(double rad) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  Vector3d FKSrv::FK(
    std::array<Eigen::Matrix3d, 6> R_leg,
    std::array<Eigen::Vector3d, 7> P_leg,
    int point
  ) {
    Vector3d fk_result;
    switch(point) {
      case 0:
        fk_result = 
            P_leg[0];
        break;
      case 1:
        fk_result = 
            R_leg[0] * P_leg[1]
          + P_leg[0];
        break;
      case 2:
        fk_result = 
            R_leg[0] * R_leg[1] * P_leg[2]
          + R_leg[0] * P_leg[1]
          + P_leg[0];
        break;
      case 3:
        fk_result = 
            R_leg[0] * R_leg[1] * R_leg[2] * P_leg[3]
          + R_leg[0] * R_leg[1] * P_leg[2]
          + R_leg[0] * P_leg[1]
          + P_leg[0];
        break;
      case 4:
        fk_result = 
            R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * P_leg[4]
          + R_leg[0] * R_leg[1] * R_leg[2] * P_leg[3]
          + R_leg[0] * R_leg[1] * P_leg[2]
          + R_leg[0] * P_leg[1]
          + P_leg[0];
        break;
      case 5:
        fk_result = 
            R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * R_leg[4] * P_leg[5]
          + R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * P_leg[4]
          + R_leg[0] * R_leg[1] * R_leg[2] * P_leg[3]
          + R_leg[0] * R_leg[1] * P_leg[2]
          + R_leg[0] * P_leg[1]
          + P_leg[0];
        break;
      case 6:
        fk_result = 
            R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * R_leg[4] * R_leg[5] * P_leg[6]
          + R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * R_leg[4] * P_leg[5]
          + R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * P_leg[4]
          + R_leg[0] * R_leg[1] * R_leg[2] * P_leg[3]
          + R_leg[0] * R_leg[1] * P_leg[2]
          + R_leg[0] * P_leg[1]
          + P_leg[0];
        break;
    }
    return (fk_result);
  }

// DEBUG===/*  脚の関節位置の読み込み。基準(0, 0, 0)は心臓の位置あたり
  void FKSrv::DEBUG_ParameterSetting() {
    P_legL_ = {  // 左脚
        Vector3d(-0.005, 0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    P_legR_ = {  // 右脚
        Vector3d(-0.005, -0.037, -0.1222),  // o(基準) -> 1
        Vector3d(0, 0, 0),  // 1 -> 2
        Vector3d(0, 0, 0),  // 2 -> 3
        Vector3d(0, 0, -0.093),  // 3 -> 4
        Vector3d(0, 0, -0.093),  // 4 -> 5
        Vector3d(0, 0, 0),  // 5 -> 6
        Vector3d(0, 0, 0)  // 6 -> a(足裏)
    };
  }
// DEBUG===*/

  // Service Server
  void FKSrv::FK_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
  ) {
    // auto time = rclcpp::Clock{}.now().seconds();
    // get values
    Q_legR_ = request->q_target_r;
    Q_legL_ = request->q_target_l;
    R_legR_ = {Rz(Q_legR_[0]), Rx(Q_legR_[1]), Ry(Q_legR_[2]), Ry(Q_legR_[3]), Ry(Q_legR_[4]), Rx(Q_legR_[5])};
    R_legL_ = {Rz(Q_legL_[0]), Rx(Q_legL_[1]), Ry(Q_legL_[2]), Ry(Q_legL_[3]), Ry(Q_legL_[4]), Rx(Q_legL_[5])};

    // function FK. get result
    FK_resultR_ = FK(R_legR_, P_legR_, request->fk_point);
    FK_resultL_ = FK(R_legL_, P_legL_, request->fk_point);
    
    // set response values
    response->p_result_r = {FK_resultR_[0], FK_resultR_[1], FK_resultR_[2]};
    response->p_result_l = {FK_resultL_[0], FK_resultL_[1], FK_resultL_[2]};
    response->q_result_r = request->q_target_r;  // FKで使わなかった値（変更なしの値）は、reqの値をそのまま返す
    response->q_result_l = request->q_target_l;

    // auto time2 = rclcpp::Clock{}.now().seconds();
    // auto time_dev = time2 - time;
    // if(time_max < time_dev){time_max = time_dev;}
    // if(time_min > time_dev){time_min = time_dev;}
    // std::cout << "[FK]: " << time_dev << "    max: " << time_max <<  "    min: " << time_min << std::endl;
    // time = time2;
  }

  // Node Setting
  FKSrv::FKSrv(
    const rclcpp::NodeOptions& options
  ) : Node("FK_SrvServer", options) {
    using namespace std::placeholders;

    // RCLCPP_INFO(this->get_logger(), "Start up FK_SrvServer. Hello FK_SrvServer!!");

// DEBUG===/*
    DEBUG_ParameterSetting();
// DEBUG===*/

    toKine_srv_ = this->create_service<msgs_package::srv::ToKinematicsMessage>(
      "FK", 
      std::bind(&FKSrv::FK_SrvServer, this, _1, _2),
      custom_qos_profile
    );

    // RCLCPP_INFO(this->get_logger(), "Waiting FK Client...");
  }
}