#ifndef ROBOT_MANAGER_PLUGIN_BASE_CONVERT_TO_JOINT_STATES_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_CONVERT_TO_JOINT_STATES_HPP

#include <vector>
#include <array>
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"

namespace control_plugin_base 
{
  struct LegJointStatesPattern {  // TODO: 実装にもよるが、片脚分だけで十分にできそう。LegStatesは片脚なので、合わせたい。
    std::vector<std::array<double, 6>> joint_ang_pat_legR;  // 6自由度未満、冗長にも対応したいので、可変長 in 可変長.
    std::vector<std::array<double, 6>> joint_ang_pat_legL;  // このファイル内で、URDFから脚の関節数を参照して固定長の長さに適用するのも面白そう。
    std::vector<std::array<double, 6>> joint_vel_pat_legR;
    std::vector<std::array<double, 6>> joint_vel_pat_legL;
  };

  class ConvertToJointStates {
    public:
      virtual std::unique_ptr<LegJointStatesPattern> convert_into_joint_states(
        const std::shared_ptr<WalkingStabilization> walking_stabilization_ptr
      ) = 0;
      virtual ~ConvertToJointStates(){}
      
    protected:
      ConvertToJointStates(){}
  };
}

#endif