#ifndef ROBOT_MANAGER_PLUGIN_BASE_FORWARD_KINEMATICS_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_FORWARD_KINEMTAICS_HPP

#include "Eigen/Dense"

// TODO: control_plugin_baseではなく、kinematics_plugin_baseとして、別pkgとして管理するべきかも
namespace control_plugin_base 
{
  // FK, IK, Jac. 全て同じ型を使うのはわかりにくい希ガス。
  // TODO: 型は全て一箇所にまとめたい。ライブラリとか。
  struct LegStates {
    Eigen::Vector3d end_eff_pos;
    Eigen::Matrix3d end_eff_rot;
    std::array<Eigen::Vector3d, 7> link_len;
    std::array<Eigen::Matrix3d, 6> joint_rot;
    std::array<Eigen::Vector3d, 6> unit_vec;
    std::array<double, 6> joint_ang;
    int8_t joint_point;  // FK用。TODO: こいつを外に出したい、別引数したい。
  };

  class ForwardKinematics {
    public:
      virtual void forward_kinematics(
        std::shared_ptr<LegStates> leg_states_ptr
      ) = 0;
      virtual ~ForwardKinematics(){}
    
    protected:
      ForwardKinematics(){}
  };
}

#endif