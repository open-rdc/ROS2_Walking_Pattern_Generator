#ifndef ROBOT_MANAGER_PLUGIN_BASE_INVERSE_KINEMATICS_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_INVERSE_KINEMATICS_HPP

namespace control_plugin_base
{
  class InverseKinemtics {
    public:
      virtual void inverse_kinematics() = 0;
      virtual ~InverseKinemtics(){}
    protected:
      InverseKinemtics(){}
  };
}

#endif