#ifndef ROBOT_MANAGER_FOOT_STEP_PLANNER_BASE_HPP
#define ROBOT_MANAGER_FOOT_STEP_PLANNER_BASE_HPP

namespace plugin_base 
{
  class FootStepPlanner {
    public:
      virtual void foot_step_planner() = 0;
      virtual ~FootStepPlanner(){}
    protected:
      FootStepPlanner(){}
  };
}

#endif 