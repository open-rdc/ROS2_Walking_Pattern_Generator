#include "kinematics/ForwardKinematics.hpp"

using namespace Eigen;

namespace kinematics
{
  // 3D Rotation Matrix
  Matrix3d Default_ForwardKinematics::IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d Default_ForwardKinematics::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d Default_ForwardKinematics::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d Default_ForwardKinematics::Rz(double rad) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  // CHECKME: あってもなくても良い。オーバーヘッドの違い次第。
  std::array<Matrix3d, 6> Default_ForwardKinematics::getR_leg(
    std::array<double, 6> Q_leg
  ) {
    return {Rz(Q_leg[0]), Rx(Q_leg[1]), Ry(Q_leg[2]), Ry(Q_leg[3]), Ry(Q_leg[4]), Rx(Q_leg[5])};
  }

  void Default_ForwardKinematics::forward_kinematics(
    std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
    Eigen::Vector3d& end_eff_pos_ptr
  ) {
    Default_ForwardKinematics::forward_kinematics(
      leg_states_ptr,
      6,
      end_eff_pos_ptr
    );
  }

  // 関数のオーバーライドをして、joint_pointを入れずに導出できるようにする。 or forで再帰的に回す。
  void Default_ForwardKinematics::forward_kinematics(
    std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
    int joint_point,
    Eigen::Vector3d& end_eff_pos_ptr
  ) {
    std::array<Eigen::Matrix3d, 6> joint_rot = Default_ForwardKinematics::getR_leg(leg_states_ptr->joint_ang);

    // std::cout << "Here is default forward kinematics class."  << std::endl;

    // CHECKME: 全て値を参照しに行っているが、値のコピーとのオーバーヘッドを比較するべき。参照のオーバーヘッドが溜まってバカにならないかも。
    switch(joint_point) {
      case 0:
        end_eff_pos_ptr = 
            leg_states_ptr->link_len[0];
        break;
      case 1:
        end_eff_pos_ptr = 
            joint_rot[0] * leg_states_ptr->link_len[1]
          + leg_states_ptr->link_len[0];
        break;
      case 2:
        end_eff_pos_ptr = 
            joint_rot[0] * joint_rot[1] * leg_states_ptr->link_len[2]
          + joint_rot[0] * leg_states_ptr->link_len[1]
          + leg_states_ptr->link_len[0];
        break;
      case 3:
        end_eff_pos_ptr = 
            joint_rot[0] * joint_rot[1] * joint_rot[2] * leg_states_ptr->link_len[3]
          + joint_rot[0] * joint_rot[1] * leg_states_ptr->link_len[2]
          + joint_rot[0] * leg_states_ptr->link_len[1]
          + leg_states_ptr->link_len[0];
        break;
      case 4:
        end_eff_pos_ptr = 
            joint_rot[0] * joint_rot[1] * joint_rot[2] * joint_rot[3] * leg_states_ptr->link_len[4]
          + joint_rot[0] * joint_rot[1] * joint_rot[2] * leg_states_ptr->link_len[3]
          + joint_rot[0] * joint_rot[1] * leg_states_ptr->link_len[2]
          + joint_rot[0] * leg_states_ptr->link_len[1]
          + leg_states_ptr->link_len[0];
        break;
      case 5:
        end_eff_pos_ptr = 
            joint_rot[0] * joint_rot[1] * joint_rot[2] * joint_rot[3] * joint_rot[4] * leg_states_ptr->link_len[5]
          + joint_rot[0] * joint_rot[1] * joint_rot[2] * joint_rot[3] * leg_states_ptr->link_len[4]
          + joint_rot[0] * joint_rot[1] * joint_rot[2] * leg_states_ptr->link_len[3]
          + joint_rot[0] * joint_rot[1] * leg_states_ptr->link_len[2]
          + joint_rot[0] * leg_states_ptr->link_len[1]
          + leg_states_ptr->link_len[0];
        break;
      case 6:
        end_eff_pos_ptr = 
            joint_rot[0] * joint_rot[1] * joint_rot[2] * joint_rot[3] * joint_rot[4] * joint_rot[5] * leg_states_ptr->link_len[6]
          + joint_rot[0] * joint_rot[1] * joint_rot[2] * joint_rot[3] * joint_rot[4] * leg_states_ptr->link_len[5]
          + joint_rot[0] * joint_rot[1] * joint_rot[2] * joint_rot[3] * leg_states_ptr->link_len[4]
          + joint_rot[0] * joint_rot[1] * joint_rot[2] * leg_states_ptr->link_len[3]
          + joint_rot[0] * joint_rot[1] * leg_states_ptr->link_len[2]
          + joint_rot[0] * leg_states_ptr->link_len[1]
          + leg_states_ptr->link_len[0];
        break;
    }
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(kinematics::Default_ForwardKinematics, control_plugin_base::ForwardKinematics)