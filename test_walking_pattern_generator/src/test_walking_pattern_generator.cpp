#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"

namespace walking_pattern_generator
{
  class LinearInvertedPendulumModel : public control_plugin_base::WalkingPatternGenerator
  {
    public:
      std::unique_ptr<control_plugin_base::WalkingPattern> walking_pattern_generator(
        const std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr
      ) override
      {
        auto walking_pattern_ptr = std::make_unique<control_plugin_base::WalkingPattern>();
        walking_pattern_ptr->cog_pos = {{1, 2, 3}};
        walking_pattern_ptr->cog_vel = {{4, 5, 6}};
        walking_pattern_ptr->zmp_pos = {{7, 8}};

        std::cout << "Here is linear inverted pendulum model class." << std::endl;

        return walking_pattern_ptr;
      }
    private:
  };
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(walking_pattern_generator::LinearInvertedPendulumModel, control_plugin_base::WalkingPatternGenerator)