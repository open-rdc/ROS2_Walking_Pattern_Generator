#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "kinematics/FK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace kinematics
{
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
    std::array<Eigen::Vector3d, 7> P_leg
  ) {
    return (
        R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * R_leg[4] * R_leg[5] * P_leg[6]
      + R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * R_leg[4] * P_leg[5]
      + R_leg[0] * R_leg[1] * R_leg[2] * R_leg[3] * P_leg[4]
      + R_leg[0] * R_leg[1] * R_leg[2] * P_leg[3]
      + R_leg[0] * R_leg[1] * P_leg[2]
      + R_leg[0] * P_leg[1]
      + P_leg[0]
    );
  }

// DEBUG===/*
  void FKSrv::DEBUG_ParameterSetting() {
    P_legL = {
        Vector3d(-0.005, 0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    P_legR = {
        Vector3d(-0.005, -0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
  }
// DEBUG===*/

  // Service Server
  void FKSrv::FK_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
  ) {
    // DEBUG=====/*
    Q_legR = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // requestから受け取りたい
    Q_legL = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // DEBUG=====*/

    Q_legR = request->q_target_r;
    Q_legL = request->q_target_l;

    R_legR = {Rz(Q_legR[0]), Rx(Q_legR[1]), Ry(Q_legR[2]), Ry(Q_legR[3]), Ry(Q_legR[4]), Rx(Q_legR[5])};
    R_legL = {Rz(Q_legL[0]), Rx(Q_legL[1]), Ry(Q_legL[2]), Ry(Q_legL[3]), Ry(Q_legL[4]), Rx(Q_legL[5])};


    FK_resultR = FK(R_legR, P_legR);
    FK_resultL = FK(R_legL, P_legL);
    
    response->p_result_r = {FK_resultR[0], FK_resultR[1], FK_resultR[2]};
    response->p_result_l = {FK_resultL[0], FK_resultL[1], FK_resultL[2]};

    // FK or IK check flag
    response->q_result_r = {999, 999, 999, 999, 999, 999};
    response->q_result_l = {999, 999, 999, 999 ,999 ,999};

    // RCLCPP_INFO(this->get_logger(), "P Result: R -> {}, L -> {}", response->p_result_r, response->p_result_l);
  }

  // Node Setting
  FKSrv::FKSrv(
    const rclcpp::NodeOptions& options
  ) : Node("FK_SrvServer", options) {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Start up FK_SrvServer. Hello FK_SrvServer!!");

// DEBUG===/*
    DEBUG_ParameterSetting();
// DEBUG===*/
    
    toKine_srv_ptr = this->create_service<msgs_package::srv::ToKinematicsMessage>(
      "FK", 
      std::bind(&FKSrv::FK_SrvServer, this, _1, _2)
    );
  }
}