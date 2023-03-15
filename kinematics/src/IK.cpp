#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "kinematics/IK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace kinematics
{
  using namespace Eigen;

  // 3D Rotation Matrix
  Matrix3d IKSrv::IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d IKSrv::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d IKSrv::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d IKSrv::Rz(double rad) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  double IKSrv::sign(double arg) {
    return((arg >= 0) - (arg < 0));  // result 1 or -1 (true == 1, false == 0)
  }

// DEBUG===/*
  void IKSrv::DEBUG_ParameterSetting() {
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

  // Service Server
  void IKSrv::IK_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
  ) {
    //IK (ROBOTIS-OP2's Leg only)
  }

  // Node Setting
  IKSrv::IKSrv(
    const rclcpp::NodeOptions& options
  ) : Node("IK", options) {
    using namespace std::placeholders;

// DEBUG===/*
    DEBUG_ParameterSetting();
// DEBUG===*/

    toKine_srv_ptr = this->create_service<msgs_package::srv::ToKinematicsMessage>(
      "IK_SrvServer",
      std::bind(&IKSrv::IK_SrvServer, this, _1, _2)
    );
  }
}