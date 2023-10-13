#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"

namespace foot_step_planner
{
  class Default_FootStepPlanner : public control_plugin_base::FootStepPlanner
  {
    public:
      Default_FootStepPlanner(){}
      ~Default_FootStepPlanner(){}
      
      std::unique_ptr<control_plugin_base::FootStep> foot_step_planner(void) override;

    private:
  };
}