#ifndef ROBOT_MANAGER_PLUGIN_BASE_INVERSE_KINEMATICS_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_INVERSE_KINEMATICS_HPP

#include "Eigen/Dense"

namespace control_plugin_base
{
  // TODO: 型は全て一箇所にまとめたい。ライブラリとか。
  // 他プラグインと型を共有するときも、PluginBase内で各々で定義する必要がある。
  struct LegStates_ToIK {
    Eigen::Vector3d end_eff_pos;
    Eigen::Matrix3d end_eff_rot;
    std::array<Eigen::Vector3d, 7> link_len;
  };

  class InverseKinematics {
    public:
      virtual void inverse_kinematics(
        const std::shared_ptr<LegStates_ToIK> leg_states_ptr,
        std::array<double, 6>& joint_ang_ptr
      ) = 0;
      virtual ~InverseKinematics(){}
    
    protected:
      InverseKinematics(){}
  };
}

#endif