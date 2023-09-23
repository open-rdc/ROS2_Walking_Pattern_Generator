#ifndef ROBOT_MANAGER_PLUGIN_BASE_FOOT_STEP_PLANNER_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_FOOT_STEP_PLANNER_HPP

namespace control_plugin_base 
{
  struct FootStep {
    int hoge;
  };
  
  class FootStepPlanner {
    public:
      virtual void foot_step_planner() = 0;
      virtual ~FootStepPlanner(){}
    protected:
      FootStepPlanner(){}
  };
}

#endif 