#ifndef ROBOT_MANAGER_PLUGIN_BASE_CONVERT_INTO_JOINT_STATES_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_CONVERT_INTO_JOINT_STATES_HPP

namespace control_plugin_base 
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