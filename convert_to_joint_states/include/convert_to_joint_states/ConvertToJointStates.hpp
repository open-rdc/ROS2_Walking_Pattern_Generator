#ifndef CONVERT_TO_JOINT_STATES_HPP
#define CONVERT_TO_JOINT_STATES_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"

// IK, JacobianのPluginをIncludeする必要がある。

#include "Eigen/Dense"

namespace convert_to_joint_states
{
  class Default_ConvertToJointStates : public control_plugin_base::ConvertToJointStates 
  {
    public:
      std::unique_ptr<control_plugin_base::LegJointStatesPattern> convert_into_joint_states(
        const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr
      ) override;
      
    private:
  };
}

#endif