#ifndef ROBOT_MANAGER_CONVERT_INTO_JOINT_STATES_BASE_HPP
#define ROBOT_MANAGER_CONVERT_INTO_JOINT_STATES_BASE_HPP

namespace plugin_base 
{
  class ConvertIntoJointStates {
    public:
      virtual void convert_into_joint_states() = 0;
      virtual ~ConvertIntoJointStates(){}
    protected:
      ConvertIntoJointStates(){}
  };
}

#endif