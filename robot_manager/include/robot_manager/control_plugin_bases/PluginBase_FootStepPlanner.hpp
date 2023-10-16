#ifndef ROBOT_MANAGER_PLUGIN_BASE_FOOT_STEP_PLANNER_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_FOOT_STEP_PLANNER_HPP

#include <vector>
#include <array>

namespace control_plugin_base 
{
  struct FootStep {
    std::vector<std::array<double, 2>> foot_pos;
    double waist_height;
    double walking_step_time;
  };

  class FootStepPlanner {
    public:
      virtual std::unique_ptr<FootStep> foot_step_planner(void) = 0;
      virtual ~FootStepPlanner(){}
      
    protected:
      FootStepPlanner(){}
  };
}

#endif 