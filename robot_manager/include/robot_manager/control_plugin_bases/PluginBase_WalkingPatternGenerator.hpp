#ifndef ROBOT_MANAGER_PLUGIN_BASE_WALKING_PATTERN_GENERATOR_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_WALKING_PATTERN_GENERATOR_HPP

#include <vector>
#include <array>

namespace control_plugin_base
{
  struct WalkingPattern {
    std::vector<std::array<double, 3>> cog_pos;
    std::vector<std::array<double, 3>> cog_vel;
    std::vector<std::array<double, 2>> zmp_pos;
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