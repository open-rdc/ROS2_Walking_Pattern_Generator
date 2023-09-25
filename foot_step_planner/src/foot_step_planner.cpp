#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"

namespace foot_step_planner
{
  class Default_FootStepPlanner : public control_plugin_base::FootStepPlanner
  {
    public:
      std::unique_ptr<control_plugin_base::FootStep> foot_step_planner(void) override {
        auto foot_step_ptr = std::make_unique<control_plugin_base::FootStep>();
        foot_step_ptr->zmp_pos = {{1, 2}};

        std::cout << "Here is default foot step planner class." << std::endl;

        return foot_step_ptr;
      }
    private:
  };
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(foot_step_planner::Default_FootStepPlanner, control_plugin_base::FootStepPlanner)