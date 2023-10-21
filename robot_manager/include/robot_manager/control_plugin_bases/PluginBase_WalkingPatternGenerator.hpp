#ifndef ROBOT_MANAGER_PLUGIN_BASE_WALKING_PATTERN_GENERATOR_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_WALKING_PATTERN_GENERATOR_HPP

#include <vector>
#include <array>
#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"

#include "Eigen/Dense"

namespace control_plugin_base
{
  struct WalkingPattern {
    std::vector<std::array<double, 3>> cc_cog_pos_ref;  // 重心位置
    std::vector<std::array<double, 3>> cc_cog_vel_ref;  // 重心速度
    // std::vector<Eigen::Vector<double, 3>> cc_foot_sup_pos_ref;  // 支持脚足先位置
    // std::vector<Eigen::Vector<double, 3>> cc_foot_swing_pos_ref;  // 遊脚足先位置
    std::vector<std::array<double, 2>> wc_foot_land_pos_ref;  // 着地位置
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