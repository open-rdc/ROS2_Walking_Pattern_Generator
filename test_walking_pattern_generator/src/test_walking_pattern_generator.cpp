#include "rclcpp/rclcpp.hpp"
#include "robot_manager/plugin_bases/WalkingPatternGenerator_base.hpp"


namespace walking_pattern_generator
{
  class LinearInvertedPendulumModel : public plugin_base::WalkingPatternGenerator
  {
    public:
      void walking_pattern_generator(
        const plugin_base::FootStep *foot_step_ptr_,
        plugin_base::WalkingPattern *walking_pattern_ptr_
      ) override
      {
        walking_pattern_ptr_->cog_pos = {{1, 2, 3}};
        walking_pattern_ptr_->cog_vel = {{4, 5, 6}};
        walking_pattern_ptr_->zmp_pos = {{7, 8}};
        std::cout << "Here is linear inverted pendulum model class." << std::endl;
      }
    private:
      plugin_base::FootStep *foot_step_ptr_;
      plugin_base::WalkingPattern *walking_pattern_ptr_;
  };
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(walking_pattern_generator::LinearInvertedPendulumModel, plugin_base::WalkingPatternGenerator)