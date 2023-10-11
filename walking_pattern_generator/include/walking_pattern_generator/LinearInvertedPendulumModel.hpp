#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"

namespace walking_pattern_generator
{
  class WPG_LinearInvertedPendulumModel : public control_plugin_base::WalkingPatternGenerator {
    public:
      std::unique_ptr<control_plugin_base::WalkingPattern> walking_pattern_generator(
        const std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr
      ) override;

      WPG_LinearInvertedPendulumModel(){}

    private:

  };
}