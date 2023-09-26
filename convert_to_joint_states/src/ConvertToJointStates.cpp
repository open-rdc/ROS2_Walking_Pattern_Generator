#include "convert_to_joint_states/ConvertToJointStates.hpp"

namespace convert_to_joint_states
{
  std::unique_ptr<control_plugin_base::LegJointStatesPattern> Default_ConvertToJointStates::convert_into_joint_states(
    const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr
  ) {
    auto leg_joint_states_pat_ptr = std::make_unique<control_plugin_base::LegJointStatesPattern>();
    leg_joint_states_pat_ptr->joint_ang_pat_legL = {{1, 2, 3, 4, 5, 6}};
    leg_joint_states_pat_ptr->joint_ang_pat_legR = {{7, 8, 9, 0, 1, 2}};
    leg_joint_states_pat_ptr->joint_vel_pat_legL = {{3, 4, 5, 6, 7, 8}};
    leg_joint_states_pat_ptr->joint_vel_pat_legR = {{9, 0, 1, 2, 3, 4}};

    std::cout << "Here is default convert to joint states class." << std::endl;

    return leg_joint_states_pat_ptr;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(convert_to_joint_states::Default_ConvertToJointStates, control_plugin_base::ConvertToJointStates)