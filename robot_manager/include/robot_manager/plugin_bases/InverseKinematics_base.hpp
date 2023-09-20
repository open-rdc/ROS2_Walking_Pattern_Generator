#ifndef ROBOT_MANAGER_INVERSE_KINEMATICS_BASE_HPP
#define ROBOT_MANAGER_INVERSE_KINEMATICS_BASE_HPP

namespace plugin_base
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