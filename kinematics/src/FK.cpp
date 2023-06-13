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

// // DEBUG===/*  脚の関節位置の読み込み。基準(0, 0, 0)は心臓の位置あたり
//   void FK::DEBUG_ParameterSetting() {
//     P_legL_ = {  // 左脚
//         Vector3d(-0.005, 0.037, -0.1222),
//         Vector3d(0, 0, 0),
//         Vector3d(0, 0, 0),
//         Vector3d(0, 0, -0.093),
//         Vector3d(0, 0, -0.093),
//         Vector3d(0, 0, 0),
//         Vector3d(0, 0, 0)
//     };
//     P_legR_ = {  // 右脚
//         Vector3d(-0.005, -0.037, -0.1222),  // o(基準) -> 1
//         Vector3d(0, 0, 0),  // 1 -> 2
//         Vector3d(0, 0, 0),  // 2 -> 3
//         Vector3d(0, 0, -0.093),  // 3 -> 4
//         Vector3d(0, 0, -0.093),  // 4 -> 5
//         Vector3d(0, 0, 0),  // 5 -> 6
//         Vector3d(0, 0, 0)  // 6 -> a(足裏)
//     };
//   }

  std::array<Matrix3d, 6> FK::getR_leg(
    std::array<double, 6> Q_leg
  ) {
    return {Rz(Q_leg[0]), Rx(Q_leg[1]), Ry(Q_leg[2]), Ry(Q_leg[3]), Ry(Q_leg[4]), Rx(Q_leg[5])};
  }


  // 関数のオーバーライドをして、joint_pointを入れずに導出できるようにする。 or forで再帰的に回す。
  Vector3d FK::getFK(
    std::array<double, 6> Q_leg,
    std::array<Eigen::Vector3d, 7> P_leg,
    int joint_point
  ) {
    R_leg_ = FK::getR_leg(Q_leg);

    Vector3d fk_result;
    switch(joint_point) {
      case 0:
        fk_result = 
            P_leg[0];
        break;
      case 1:
        fk_result = 
            R_leg_[0] * P_leg[1]
          + P_leg[0];
        break;
      case 2:
        fk_result = 
            R_leg_[0] * R_leg_[1] * P_leg[2]
          + R_leg_[0] * P_leg[1]
          + P_leg[0];
        break;
      case 3:
        fk_result = 
            R_leg_[0] * R_leg_[1] * R_leg_[2] * P_leg[3]
          + R_leg_[0] * R_leg_[1] * P_leg[2]
          + R_leg_[0] * P_leg[1]
          + P_leg[0];
        break;
      case 4:
        fk_result = 
            R_leg_[0] * R_leg_[1] * R_leg_[2] * R_leg_[3] * P_leg[4]
          + R_leg_[0] * R_leg_[1] * R_leg_[2] * P_leg[3]
          + R_leg_[0] * R_leg_[1] * P_leg[2]
          + R_leg_[0] * P_leg[1]
          + P_leg[0];
        break;
      case 5:
        fk_result = 
            R_leg_[0] * R_leg_[1] * R_leg_[2] * R_leg_[3] * R_leg_[4] * P_leg[5]
          + R_leg_[0] * R_leg_[1] * R_leg_[2] * R_leg_[3] * P_leg[4]
          + R_leg_[0] * R_leg_[1] * R_leg_[2] * P_leg[3]
          + R_leg_[0] * R_leg_[1] * P_leg[2]
          + R_leg_[0] * P_leg[1]
          + P_leg[0];
        break;
      case 6:
        fk_result = 
            R_leg_[0] * R_leg_[1] * R_leg_[2] * R_leg_[3] * R_leg_[4] * R_leg_[5] * P_leg[6]
          + R_leg_[0] * R_leg_[1] * R_leg_[2] * R_leg_[3] * R_leg_[4] * P_leg[5]
          + R_leg_[0] * R_leg_[1] * R_leg_[2] * R_leg_[3] * P_leg[4]
          + R_leg_[0] * R_leg_[1] * R_leg_[2] * P_leg[3]
          + R_leg_[0] * R_leg_[1] * P_leg[2]
          + R_leg_[0] * P_leg[1]
          + P_leg[0];
        break;
    }
    return (fk_result);
  }

  FK::FK(
    const rclcpp::NodeOptions& options
  ) : Node("FK", options) {

// DEBUG===/*
      // DEBUG_ParameterSetting();  // 共有ライブラリって、動的に読み込まれるのか？毎回、一から実行するなら、このパラメータも引数にしてしまったほうが良いのではないか？
      // // -> オブジェクトを生成したときに、コンストラクタが実行される。Classの関数を使いたいときには、はじめに以下のようにオブジェクトを生成する必要がある。
      // // FK FK();  // class名（＝型） 変数名();  引数を与える必要があるかもしれん。
// DEBUG===*/

  }

}