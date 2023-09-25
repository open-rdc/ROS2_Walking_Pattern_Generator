#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"

namespace walking_stabilization_controller
{
  class Default_WalkingStabilizationController : public control_plugin_base::WalkingStabilizationController
  {
    public:
      std::unique_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_controller(
        std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr
      ) override;
    private:
  };
}