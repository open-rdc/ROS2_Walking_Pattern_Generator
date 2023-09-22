#ifndef ROBOT_MANAGER_WALKING_PATTERN_GENERATOR_BASE_HPP
#define ROBOT_MANAGER_WALKING_PATTERN_GENERATOR_BASE_HPP

#include <vector>
#include <array>

namespace plugin_base
{
  struct WalkingPattern {
    std::vector<std::array<double, 3>> cog_pos;
    std::vector<std::array<double, 3>> cog_vel;
    std::vector<std::array<double, 2>> zmp_pos;
  }; 
  
  class WalkingPatternGenerator {
    public:
      virtual void walking_pattern_generator(
        const struct FootStep *foot_step_ptr_,
        struct WalkingPattern *walking_pattern_ptr_
      );
      virtual ~WalkingPatternGenerator(){}

    protected:
      WalkingPatternGenerator(){}
  };
}

#endif