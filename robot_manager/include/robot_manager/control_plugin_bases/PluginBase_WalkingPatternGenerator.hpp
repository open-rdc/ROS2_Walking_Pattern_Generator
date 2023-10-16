#ifndef ROBOT_MANAGER_PLUGIN_BASE_WALKING_PATTERN_GENERATOR_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_WALKING_PATTERN_GENERATOR_HPP

#include <vector>
#include <array>
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"

namespace control_plugin_base
{
  struct WalkingPattern {
    std::vector<std::array<double, 3>> cog_pos_ref;
    std::vector<std::array<double, 3>> cog_vel_ref;
    std::vector<std::array<double, 2>> foot_pos_ref;
  }; 
  
  class WalkingPatternGenerator {
    public:
      virtual std::unique_ptr<WalkingPattern> walking_pattern_generator(
        const std::shared_ptr<FootStep> foot_step_ptr  //  output from foot_step_planner
      ) = 0;
      virtual ~WalkingPatternGenerator(){}

    protected:
      WalkingPatternGenerator(){}
  };
}

#endif