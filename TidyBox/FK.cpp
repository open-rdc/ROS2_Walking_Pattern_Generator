#include "rclcpp/rclcpp.hpp"
// #include "/*PackageName*//srv/ToKinematics_msgs.msg"
// #include "/*PackageName*//FK.hpp"

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

  // Service Server
  void FK_SrvServer(
    const std::shared_ptr</*PackageName*/::srv::ToKinematics_msgs::Request> request,
    std::shared_ptr</*PackageName*/::srv::ToKinematics_msgs::Response> response
  ) {
    //FK (ROBOTIS-OP2's Leg only)
  }

  // Node Setting
  FKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
   : Node("FK", options) {
    srv_ptr = this->create_service</*PackageName*/::srv::ToKinematics_msgs>(/**/);
  }
}