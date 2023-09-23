#ifndef ROBOT_MANAGER_PLUGIN_BASE_FORWARD_KINEMATICS_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_FORWARD_KINEMTAICS_HPP

namespace control_plugin_base 
{
  class ForwardKinematics {
    public:
      virtual void forward_kinematics() = 0;
      virtual ~ForwardKinematics(){}
    protected:
      ForwardKinematics(){}
  };
}

#endif