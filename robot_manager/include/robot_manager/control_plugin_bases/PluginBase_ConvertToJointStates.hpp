#ifndef ROBOT_MANAGER_PLUGIN_BASE_CONVERT_TO_JOINT_STATES_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_CONVERT_TO_JOINT_STATES_HPP

namespace control_plugin_base 
{
  class ConvertToJointStates {
    public:
      virtual void convert_into_joint_states() = 0;
      virtual ~ConvertToJointStates(){}
    protected:
      ConvertToJointStates(){}
  };
}

#endif