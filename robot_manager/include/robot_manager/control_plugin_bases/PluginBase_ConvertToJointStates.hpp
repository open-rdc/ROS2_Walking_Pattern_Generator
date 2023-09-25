#ifndef ROBOT_MANAGER_PLUGIN_BASE_CONVERT_TO_JOINT_STATES_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_CONVERT_TO_JOINT_STATES_HPP

#include <vector>
#include <array>
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"

namespace control_plugin_base 
{
  struct LegJointStates {
    std::vector<std::vector<double>> joint_ang_legR;  // 6自由度未満、冗長にも対応したいので、可変長 in 可変長.
    std::vector<std::vector<double>> joint_ang_legL;  // このファイル内で、URDFから脚の関節数を参照して固定長の長さに適用するのも面白そう。
    std::vector<std::vector<double>> joint_vel_legR;
    std::vector<std::vector<double>> joint_vel_legL;
  };

  class ConvertToJointStates {
    public:
      virtual std::unique_ptr<LegJointStates> convert_into_joint_states(
        const std::shared_ptr<WalkingStabilization> walking_stabilization_ptr
      ) = 0;
      virtual ~ConvertToJointStates(){}
      
    protected:
      ConvertToJointStates(){}
  };
}

#endif