#include "foot_step_planner/foot_step_planner.hpp"

namespace foot_step_planner
{
  std::unique_ptr<control_plugin_base::FootStep> Default_FootStepPlanner::foot_step_planner(void) {
    auto foot_step_ptr = std::make_unique<control_plugin_base::FootStep>();
    
    return foot_step_ptr;
  }
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(foot_step_planner::Default_FootStepPlanner, control_plugin_base::FootStepPlanner)