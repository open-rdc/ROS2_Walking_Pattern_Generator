#ifndef ROBOT_MANAGER_PLUGIN_BASE_WALKING_STABILIZATION_CONTROLLER_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_WALKING_STABILIZATION_CONTROLLER_HPP

#include <vector>
#include <array>
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"

namespace control_plugin_base
{
  struct WalkingStabilization {
    std::vector<std::array<double, 3>> cog_pos_fix;
    std::vector<std::array<double, 3>> cog_vel_fix;
    std::vector<std::array<double, 2>> zmp_pos_fix;
  };

  class WalkingStabilizationController {
    public:
      virtual std::unique_ptr<WalkingStabilization> walking_stabilization_controller(
        const std::shared_ptr<WalkingPattern> walking_pattern_ptr
      ) = 0;
      virtual ~WalkingStabilizationController(){}

    protected:
      WalkingStabilizationController(){}
  };
  // struct WalkingStabilization {
  //   std::array<double, 3> cog_pos_fix;
  //   std::array<double, 3> cog_vel_fix;
  //   std::array<double, 2> zmp_pos_fix;
  // };

  // class WalkingStabilizationController {
  //   public:
  //     virtual std::unique_ptr<WalkingStabilization> walking_stabilization_controller(
  //       const std::shared_ptr<WalkingPattern> walking_pattern_ptr,
  //       const uint32_t control_step
  //     ) = 0;
  //     virtual ~WalkingStabilizationController(){}

  //   protected:
  //     WalkingStabilizationController(){}
  // };
}

#endif