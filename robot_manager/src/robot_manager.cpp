#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<control_plugin_base::WalkingPatternGenerator> wpg_loader("robot_manager", "control_plugin_base::WalkingPatternGenerator");
  pluginlib::ClassLoader<control_plugin_base::FootStepPlanner> fsp_loader("robot_manager", "control_plugin_base::FootStepPlanner");

  try
  {
    std::shared_ptr<control_plugin_base::WalkingPatternGenerator> wpg = wpg_loader.createSharedInstance("walking_pattern_generator::LinearInvertedPendulumModel");
    std::shared_ptr<control_plugin_base::FootStepPlanner> fsp = fsp_loader.createSharedInstance("foot_step_planner::Default_FootStepPlanner");

    std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr = fsp->foot_step_planner();

    std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = wpg->walking_pattern_generator(foot_step_ptr);
    
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("ERROR!!: %s\n", ex.what());
  }
  
  return 0;
}