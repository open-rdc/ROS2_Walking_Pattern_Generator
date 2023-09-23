#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<control_plugin_base::WalkingPatternGenerator> test_loader("robot_manager", "control_plugin_base::WalkingPatternGenerator");

  try
  {
    std::shared_ptr<control_plugin_base::WalkingPatternGenerator> test_WPG = test_loader.createSharedInstance("walking_pattern_generator::LinearInvertedPendulumModel");
    auto foot_step_ptr_ = std::make_shared<control_plugin_base::FootStep>();

    std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = test_WPG->walking_pattern_generator(foot_step_ptr_);
    
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("ERROR!!: %s\n", ex.what());
  }
  
  return 0;
}