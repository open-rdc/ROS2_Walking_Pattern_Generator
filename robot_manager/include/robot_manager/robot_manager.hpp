#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"

#include "Eigen/Dense"

namespace robot_manager
{
  class RobotManager {
    public:
      RobotManager();
      ~RobotManager(){}

      void Step();
    
    private:
      std::shared_ptr<control_plugin_base::WalkingPatternGenerator> wpg;
      std::shared_ptr<control_plugin_base::FootStepPlanner> fsp;
      std::shared_ptr<control_plugin_base::WalkingStabilizationController> wsc;
      std::shared_ptr<control_plugin_base::ConvertToJointStates> ctjs;

  };
}