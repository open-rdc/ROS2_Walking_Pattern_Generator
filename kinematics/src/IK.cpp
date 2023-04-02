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

  Vector3d IKSrv::Array2Vector(std::array<double, 3> array) {
    return {array[0], array[1], array[2]};
  }
  Matrix3d IKSrv::Array2Matrix(std::array<double, 9> array) {
    Matrix3d R;
    R << array[0], array[1], array[2],
         array[3], array[4], array[5],
         array[6], array[7], array[8];
    return R;
  }

  //IK (ROBOTIS-OP2's Leg only. analytical method)
  std::array<double, 6> IKSrv::IK(
    std::array<Eigen::Vector3d, 7> P_leg,
    Eigen::Vector3d P_target_leg,
    Eigen::Matrix3d R_target_leg
  ) {
    std::array<double, 6> Q;
    Vector3d P_target_leg_zhip2end;  // 股関節(z軸関節)から末端まで
    P_target_leg_zhip2end = R_target_leg.transpose() * (P_leg[0] - P_target_leg);

    double a, b, c;
    a = abs(P_leg[3](2));
    b = abs(P_leg[4](2));
    c = sqrt(pow(P_target_leg_zhip2end(0), 2) + pow(P_target_leg_zhip2end(1), 2) + pow(P_target_leg_zhip2end(2), 2));  // pow(A, B) == A^B =- AのB乗

    Q[3] = -1 * acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b)) + pi_;

    double d;
    d = asin((a * sin(pi_ - Q[3])) / c);

    Q[4] = -1 * atan2(P_target_leg_zhip2end(0), sign(P_target_leg_zhip2end(2)) * sqrt(pow(P_target_leg_zhip2end(1), 2) + pow(P_target_leg_zhip2end(2), 2))) - d;
    Q[5] = atan2(P_target_leg_zhip2end(1), P_target_leg_zhip2end(2));

    Matrix3d R_target_leg_begin2yhip;  // 基準から股関節(y軸関節)まで
    R_target_leg_begin2yhip = R_target_leg * Rx(Q[5]).transpose() * Ry(Q[4]).transpose() * Ry(Q[3]).transpose();

    Q[0] = atan2(-R_target_leg_begin2yhip(0, 1), R_target_leg_begin2yhip(1, 1));
    Q[1] = atan2(R_target_leg_begin2yhip(2, 1), -R_target_leg_begin2yhip(0, 1) * sin(Q[0]) + R_target_leg_begin2yhip(1, 1) * cos(Q[0]));
    Q[2] = atan2(-R_target_leg_begin2yhip(2, 0), R_target_leg_begin2yhip(2, 2));

    return Q;
  }

// DEBUG===/*
  void IKSrv::DEBUG_ParameterSetting() {
    P_legL_ = {
        Vector3d(-0.005, 0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    P_legR_ = {
        Vector3d(-0.005, -0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
  }
// DEBUG=====*/

  // Service Server
  void IKSrv::IK_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
  ) {
    // requestを記録（std::arrayからEigenに変換）
    P_target_legR_ = Array2Vector(request->p_target_r);
    R_target_legR_ = Array2Matrix(request->r_target_r);
    P_target_legL_ = Array2Vector(request->p_target_l);
    R_target_legL_ = Array2Matrix(request->r_target_l);

    response->q_result_r = IK(P_legR_, P_target_legR_, R_target_legR_);
    response->q_result_l = IK(P_legL_, P_target_legL_, R_target_legL_);
    // IKで求めなかった値は、reqをそのままresに渡す
    response->p_result_r = request->p_target_r;
    response->p_result_l = request->p_target_l;
  }

  // Node Setting
  IKSrv::IKSrv(
    const rclcpp::NodeOptions& options
  ) : Node("IK_SrvServer", options) {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Start up IK_SrvServer. Hello IK_SrvServer!!");

// DEBUG===/*
    DEBUG_ParameterSetting();
// DEBUG===*/

    toKine_srv_ = this->create_service<msgs_package::srv::ToKinematicsMessage>(
      "IK",
      std::bind(&IKSrv::IK_SrvServer, this, _1, _2)
    );

    RCLCPP_INFO(this->get_logger(), "Waiting IK Client...");
  }
}