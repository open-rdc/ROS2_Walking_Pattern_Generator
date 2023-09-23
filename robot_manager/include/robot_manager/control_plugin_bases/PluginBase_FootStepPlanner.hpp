#ifndef ROBOT_MANAGER_PLUGIN_BASE_FOOT_STEP_PLANNER_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_FOOT_STEP_PLANNER_HPP

#include <vector>
#include <array>

namespace control_plugin_base 
{
  struct FootStep {
    std::vector<std::array<double, 2>> zmp_pos;
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