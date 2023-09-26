#include "rclcpp/rclcpp.hpp"
#include "kinematics/IK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

using namespace Eigen;

namespace kinematics
{
// DEBUG===/*
  // void IK::DEBUG_ParameterSetting() {
  //   P_legL_ = {
  //       Vector3d(-0.005, 0.037, -0.1222),
  //       Vector3d(0, 0, 0),
  //       Vector3d(0, 0, 0),
  //       Vector3d(0, 0, -0.093),
  //       Vector3d(0, 0, -0.093),
  //       Vector3d(0, 0, 0),
  //       Vector3d(0, 0, 0)
  //   };
  //   P_legR_ = {
  //       Vector3d(-0.005, -0.037, -0.1222),
  //       Vector3d(0, 0, 0),
  //       Vector3d(0, 0, 0),
  //       Vector3d(0, 0, -0.093),
  //       Vector3d(0, 0, -0.093),
  //       Vector3d(0, 0, 0),
  //       Vector3d(0, 0, 0)
  //   };
  // }
// DEBUG=====*/

  Vector3d IK::Array2Vector(std::array<double, 3> array) {
    return {array[0], array[1], array[2]};
  }
  Matrix3d IK::Array2Matrix(std::array<double, 9> array) {
    Matrix3d R;
    R << array[0], array[1], array[2],
         array[3], array[4], array[5],
         array[6], array[7], array[8];
    return R;
  }

  Matrix3d IK::IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d IK::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d IK::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d IK::Rz(double rad) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  double IK::sign(double arg) {
    return((arg >= 0) - (arg < 0));  // result 1 or -1 (true == 1, false == 0)
  }

  //IK (ROBOTIS-OP2's Leg only. analytical method)
  std::array<double, 6> IK::getIK(
    std::array<Eigen::Vector3d, 7> P_leg,
    Eigen::Vector3d P_target_leg,
    Eigen::Matrix3d R_target_leg
  ) {
    std::array<double, 6> Q;
    Vector3d P_target_leg_zhip2end;  // 股関節(z軸関節)から末端まで
    P_target_leg_zhip2end = R_target_leg.transpose() * (P_leg[0] - P_target_leg);

    double a, b, c;
    a = std::abs(P_leg[3](2));
    b = std::abs(P_leg[4](2));
    c = std::sqrt(std::pow(P_target_leg_zhip2end(0), 2) + std::pow(P_target_leg_zhip2end(1), 2) + std::pow(P_target_leg_zhip2end(2), 2));  // pow(A, B) == A^B =- AのB乗

    Q[3] = -1 * std::acos((std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2)) / (2 * a * b)) + pi_;

    double d;
    d = std::asin((a * sin(pi_ - Q[3])) / c);

    Q[4] = -1 * std::atan2(P_target_leg_zhip2end(0), sign(P_target_leg_zhip2end(2)) * std::sqrt(std::pow(P_target_leg_zhip2end(1), 2) + std::pow(P_target_leg_zhip2end(2), 2))) - d;
    Q[5] = std::atan2(P_target_leg_zhip2end(1), P_target_leg_zhip2end(2));

    Matrix3d R_target_leg_begin2yhip;  // 基準から股関節(y軸関節)まで
    R_target_leg_begin2yhip = R_target_leg * Rx(Q[5]).inverse() * Ry(Q[3]+Q[4]).inverse();
    // std::cout << R_target_leg_begin2yhip << std::endl;
    // std::cout << R_target_leg_begin2yhip(0, 1) << std::endl;
    Q[0] = std::atan2(-R_target_leg_begin2yhip(0, 1), R_target_leg_begin2yhip(1, 1));
    Q[1] = std::atan2(R_target_leg_begin2yhip(2, 1), -R_target_leg_begin2yhip(0, 1) * std::sin(Q[0]) + R_target_leg_begin2yhip(1, 1) * std::cos(Q[0]));
    Q[2] = std::atan2(-R_target_leg_begin2yhip(2, 0), R_target_leg_begin2yhip(2, 2));

    return Q;
  }

  IK::IK(
    const rclcpp::NodeOptions& options
  ) : Node("IK", options) {

// DEBUG===/*
    // DEBUG_ParameterSetting();
// DEBUG===*/

  }

}