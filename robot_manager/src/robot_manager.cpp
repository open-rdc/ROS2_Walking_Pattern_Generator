#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/plugin_bases/WalkingPatternGenerator_base.hpp"

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<plugin_base::WalkingPatternGenerator> test_loader("robot_manager", "plugin_base::WalkingPatternGenerator");

  try
  {
    std::shared_ptr<plugin_base::WalkingPatternGenerator> test_WPG = test_loader.createSharedInstance("walking_pattern_generator::LinearInvertedPendulumModel");
    test_WPG->walking_pattern_generator();
  }
  catch(pluginlib::PluginlibException& ex)
  {
    std::cout << "ERROR! : " << ex.what() << std::endl;
  }
  
  return 0;
}