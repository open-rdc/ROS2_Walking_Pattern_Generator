#include "rclcpp/rclcpp.hpp"
#include "robot_manager/plugin_bases/WalkingPatternGenerator_base.hpp"


namespace walking_pattern_generator
{
  class LinearInvertedPendulumModel : public plugin_base::WalkingPatternGenerator
  {
    public:
    void walking_pattern_generator() override
      {
        std::cout << "Here is linear inverted pendulum model class." << std::endl;
      }
  };
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(walking_pattern_generator::LinearInvertedPendulumModel, plugin_base::WalkingPatternGenerator)