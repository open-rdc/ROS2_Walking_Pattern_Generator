#include "rclcpp/rclcpp.hpp"
#include "kinematics/FK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

using namespace Eigen;

namespace kinematics
{
  // 3D Rotation Matrix
  Matrix3d FK::IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d FK::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d FK::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d FK::Rz(double rad) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

// DEBUG===/*  脚の関節位置の読み込み。基準(0, 0, 0)は心臓の位置あたり
  void FK::DEBUG_ParameterSetting() {
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

  Vector3d FK::FK_calc(
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

  FK::FK(
    const rclcpp::NodeOptions& options
  ) : Node("FK", options) {

// DEBUG===/*
      void DEBUG_ParameterSetting(void);  // 共有ライブラリって、動的に読み込まれるのか？毎回、一から実行するなら、このパラメータも引数にしてしまったほうが良いのではないか？
// DEBUG===*/

  }

}