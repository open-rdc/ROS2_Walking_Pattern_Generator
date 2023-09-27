#ifndef ROBOT_MANAGER_PLUGIN_BASE_JACOBIAN_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_JACOBIAN_HPP

#include "Eigen/Dense"

namespace control_plugin_base
{
  // FK, IK, Jac. 全て同じ型を使うのはわかりにくい希ガス。
  // TODO: 型は全て一箇所にまとめたい。ライブラリとか。
  // Kinematicsは様々な箇所で用いられるので、構造体ではなく１つの変数のunique_ptrを返す形でいく。
  // 基本的にPluginの引数は、独自の構造体のConstとする。
  struct LegStates_ToJac {
    // Eigen::Vector3d end_eff_pos;
    // Eigen::Matrix3d end_eff_rot;
    std::array<Eigen::Vector3d, 7> link_len;
    // std::array<Eigen::Matrix3d, 6> joint_rot;
    std::array<Eigen::Vector3d, 6> unit_vec;
    std::array<double, 6> joint_ang;
    // int8_t joint_point;  // FK用
  };

  class Jacobian {
    public:
      virtual void jacobian(
        const std::shared_ptr<LegStates_ToJac> leg_states_jac_ptr,
        Eigen::Matrix<double, 6, 6>& leg_jacobian
      ) = 0;
      virtual ~Jacobian(){}
    
    protected:
      Jacobian(){}
  };
}

#endif