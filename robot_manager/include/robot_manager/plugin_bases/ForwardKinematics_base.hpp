#ifndef ROBOT_MANAGER_FORWARD_KINEMATICS_BASE_HPP
#define ROBOT_MANAGER_FORWARD_KINEMTAICS_BASE_HPP

namespace plugin_base 
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