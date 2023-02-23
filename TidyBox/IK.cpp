#include "rclcpp/rclcpp.hpp"
// #include "/*PackageName*//srv/ToKinematics_msgs.msg"
// #include "/*PackageName*//IK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace Kinematics
{
  using namespace IKSrv;
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

  double sign(double arg = 0) {
    return((arg >= 0) - (arg < 0))  // result 1 or -1 (true == 1, false == 0)
  }

  // Service Server
  void IK_SrvServer(
    const std::shared_ptr</*PackageName*/::srv::ToKinematics_msgs::Request> request,
    std::shared_ptr</*PackageName*/::srv::ToKinematics_msgs::Response> response
  ) {
    //IK (ROBOTIS-OP2's Leg only)
  }

  // Node Setting
  IKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
   : Node("IK", options) {
    toKine_srv_ptr = this->create_service</*PackageName*/::srv::ToKinematics_msgs>(/**/);
  }
}