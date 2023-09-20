#ifndef ROBOT_MANAGER_WALKING_PATTERN_GENERATOR_BASE_HPP
#define ROBOT_MANAGER_WALKING_PATTERN_GENERATOR_BASE_HPP

namespace plugin_base
{
  class WalkingPatternGenerator {
    public:
      virtual void walking_pattern_generator() = 0;
      virtual ~WalkingPatternGenerator(){}
    protected:
      WalkingPatternGenerator(){}
  };
}

#endif