#include "convert_to_joint_states/ConvertToJointStates.hpp"

namespace convert_to_joint_states
{
  std::unique_ptr<control_plugin_base::LegJointStates> Default_ConvertToJointStates::convert_into_joint_states(
    const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr
  ) {
    auto leg_joint_states_ptr = std::make_unique<control_plugin_base::LegJointStates>();
    leg_joint_states_ptr->joint_ang_legL = {{1, 2, 3, 4, 5, 6}};
    leg_joint_states_ptr->joint_ang_legR = {{7, 8, 9, 0, 1, 2}};
    leg_joint_states_ptr->joint_vel_legL = {{3, 4, 5, 6, 7, 8}};
    leg_joint_states_ptr->joint_vel_legR = {{9, 0, 1, 2, 3, 4}};

    std::cout << "Here is default walking stabilization controller class." << std::endl;

    return leg_joint_states_ptr;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(convert_to_joint_states::Default_ConvertToJointStates, control_plugin_base::ConvertToJointStates)