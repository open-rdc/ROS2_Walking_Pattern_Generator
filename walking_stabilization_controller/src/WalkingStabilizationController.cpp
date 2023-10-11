#include "walking_stabilization_controller/WalkingStabilizationController.hpp"

namespace walking_stabilization_controller 
{
  std::unique_ptr<control_plugin_base::WalkingStabilization> Default_WalkingStabilizationController::walking_stabilization_controller(
    const std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr
  ) {
    auto walking_stabilization_ptr = std::make_unique<control_plugin_base::WalkingStabilization>();
    walking_stabilization_ptr->cog_pos_fix = walking_pattern_ptr->cog_pos;
    walking_stabilization_ptr->cog_vel_fix = walking_pattern_ptr->cog_vel;
    walking_stabilization_ptr->zmp_pos_fix = walking_pattern_ptr->zmp_pos;

    std::cout << "Here is default walking stabilization controller class." << std::endl;

    return walking_stabilization_ptr;
  }
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(walking_stabilization_controller::Default_WalkingStabilizationController, control_plugin_base::WalkingStabilizationController)