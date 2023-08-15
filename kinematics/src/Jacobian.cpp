#include "kinematics/Jacobian.hpp"

#include "rclcpp/rclcpp.hpp"
#include "kinematics/visibility_control.h"

#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

using namespace Eigen;

namespace kinematics
{
  Matrix<double, 6, 6> Jacobian::JacobiMatrix_leg(
    std::array<double, 6> Q_leg,
    std::array<Vector3d, 6> UnitVec_leg,
    std::array<Vector3d, 7> P_leg
  ) {

    Matrix<double, 6, 6> Jacobi_leg = MatrixXd::Zero(6, UnitVec_leg.max_size());

    std::array<Eigen::Vector3d, 6> P_FK_leg;
    for(int joint_point = 0; joint_point < int(UnitVec_leg.max_size()); joint_point++) {
      P_FK_leg[joint_point] = FK_.getFK(Q_leg, P_leg, joint_point);
      // std::cout << P_FK_leg[joint_point] << std::endl;
    }

    Vector3d mat_leg = Vector3d::Zero(3);
    Vector3d pt_P_leg = Vector3d::Zero(3);
    for(int tag = 0; tag < int(UnitVec_leg.max_size()); tag++) {
      if(tag == int(UnitVec_leg.max_size()-1)) {
        mat_leg = Vector3d::Zero(3);
      }
      else { 
        // P_FK_leg[int(UnitVec_legR_.max_size())-1]: 股関節の座標を取得（基準座標から股関節までの距離を含まないFKの結果を取得）
        pt_P_leg = P_FK_leg[int(UnitVec_leg.max_size())-1] - P_FK_leg[tag];
        // std::cout << "pt_P_leg: " << pt_P_leg.transpose() << ", " << P_FK_leg[int(UnitVec_leg.max_size())-1].transpose() << ", " << P_FK_leg[tag].transpose() << std::endl;
        mat_leg = UnitVec_leg[tag].cross(pt_P_leg);
      }

      for(int i = 0; i < 3; i++) {
        Jacobi_leg(i, tag) = mat_leg[i];
        Jacobi_leg(i+3, tag) = UnitVec_leg[tag][i];
      }
    }

    return Jacobi_leg;
  }

  Jacobian::Jacobian(
    const rclcpp::NodeOptions& options
  ) : Node("Jacobian", options) {}
}