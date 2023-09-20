#ifndef ROBOT_MANAGER_WALKING_STABILIZATION_CONTROLLER_BASE_HPP
#define ROBOT_MANAGER_WALKING_STABILIZATION_CONTROLLER_BASE_HPP

namespace plugin_base
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