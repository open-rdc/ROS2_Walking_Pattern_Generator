#ifndef ROBOT_MANAGER_PLUGIN_BASE_INVERSE_KINEMATICS_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_INVERSE_KINEMATICS_HPP

#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"  // 型がほしいのでinclude
#include "robot_manager/control_plugin_bases/PluginBase_InverseKinematics.hpp"

namespace control_plugin_base
{
  class InverseKinematics {
    public:
      virtual void inverse_kinematics(
        std::shared_ptr<LegStates> leg_states_ptr
      ) = 0;
      virtual ~InverseKinematics(){}
    
    protected:
      InverseKinematics(){}
  };
}

#endif