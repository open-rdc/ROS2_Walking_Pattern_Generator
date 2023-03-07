#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "Kinematics/FK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace Kinematics
{
  using namespace FKSrv;
  using namespace Eigen;

  // 3D Rotation Matrix
  Matrix3d IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d Rx(double rad = 0) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d Ry(double rad = 0) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d Rz(double rad = 0) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  void DEBUG_ParameterSetting() {
    P_legL = {(-0.005, 0.037, -0.1222),
         (0, 0, 0),
         (0, 0, 0),
         (0, 0, -0.093),
         (0, 0, -0.093),
         (0, 0, 0),
         (0, 0, 0)
    }
    P_legR = {(-0.005, -0.037, -0.1222),
         (0, 0, 0),
         (0, 0, 0),
         (0, 0, -0.093),
         (0, 0, -0.093),
         (0, 0, 0),
         (0, 0, 0)
    }
  }

  // Service Server
  void FK_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
  ) {
    //FK (ROBOTIS-OP2's Leg only)
  }

  // Node Setting
  FKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
   : Node("FK", options) {
    DEBUG_ParameterSetting();
    toKine_srv_ptr = this->create_service<msgs_package::srv::ToKinematicsMessage>(/**/);
  }
}