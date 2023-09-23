#ifndef ROBOT_MANAGER_PLUGIN_BASE_WALKING_STABILIZATION_CONTROLLER_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_WALKING_STABILIZATION_CONTROLLER_HPP

namespace control_plugin_base
{
  class WalkingStabilizationController {
    public:
      virtual void walking_stabilization_controller() = 0;
      virtual ~WalkingStabilizationController(){}
    protected:
      WalkingStabilizationController(){}
  };
}

#endif