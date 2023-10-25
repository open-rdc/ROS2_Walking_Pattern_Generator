#include "kinematics/InverseKinematics.hpp"

using namespace Eigen;

namespace kinematics
{
  Vector3d Default_InverseKinematics::Array2Vector(std::array<double, 3> array) {
    return {array[0], array[1], array[2]};
  }
  Matrix3d Default_InverseKinematics::Array2Matrix(std::array<double, 9> array) {
    Matrix3d R;
    R << array[0], array[1], array[2],
         array[3], array[4], array[5],
         array[6], array[7], array[8];
    return R;
  }

  Matrix3d Default_InverseKinematics::IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d Default_InverseKinematics::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d Default_InverseKinematics::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d Default_InverseKinematics::Rz(double rad) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  double Default_InverseKinematics::sign(double arg) {
    return((arg >= 0) - (arg < 0));  // result 1 or -1 (true == 1, false == 0)
  }

  // IK (ROBOTIS-OP2's Leg only. analytical method)
  void Default_InverseKinematics::inverse_kinematics(
    const std::shared_ptr<control_plugin_base::LegStates_ToIK> leg_states_ptr,
    std::array<double, 6>& joint_ang_ptr
  ) {
    std::array<double, 6> Q;
    Vector3d P_target_leg_zhip2end;  // 股関節(z軸関節)から末端まで
    P_target_leg_zhip2end = leg_states_ptr->end_eff_rot.transpose() * (leg_states_ptr->link_len[0] - leg_states_ptr->end_eff_pos);

    double a, b, c;
    a = std::abs(leg_states_ptr->link_len[3](2));
    b = std::abs(leg_states_ptr->link_len[4](2));
    c = std::sqrt(std::pow(P_target_leg_zhip2end(0), 2) + std::pow(P_target_leg_zhip2end(1), 2) + std::pow(P_target_leg_zhip2end(2), 2));  // pow(A, B) == A^B =- AのB乗

    Q[3] = -1 * std::acos((std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2)) / (2 * a * b)) + pi_;

    double d;
    d = std::asin((a * sin(pi_ - Q[3])) / c);

    Q[4] = -1 * std::atan2(P_target_leg_zhip2end(0), sign(P_target_leg_zhip2end(2)) * std::sqrt(std::pow(P_target_leg_zhip2end(1), 2) + std::pow(P_target_leg_zhip2end(2), 2))) - d;
    Q[5] = std::atan2(P_target_leg_zhip2end(1), P_target_leg_zhip2end(2));

    Matrix3d R_target_leg_begin2yhip;  // 基準から股関節(y軸関節)まで
    R_target_leg_begin2yhip = leg_states_ptr->end_eff_rot * Rx(Q[5]).inverse() * Ry(Q[3]+Q[4]).inverse();
    // std::cout << R_target_leg_begin2yhip << std::endl;
    // std::cout << R_target_leg_begin2yhip(0, 1) << std::endl;
    Q[0] = std::atan2(-R_target_leg_begin2yhip(0, 1), R_target_leg_begin2yhip(1, 1));
    Q[1] = std::atan2(R_target_leg_begin2yhip(2, 1), -R_target_leg_begin2yhip(0, 1) * std::sin(Q[0]) + R_target_leg_begin2yhip(1, 1) * std::cos(Q[0]));
    Q[2] = std::atan2(-R_target_leg_begin2yhip(2, 0), R_target_leg_begin2yhip(2, 2));

    joint_ang_ptr = Q;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(kinematics::Default_InverseKinematics, control_plugin_base::InverseKinematics)