#ifndef CONVERT_TO_JOINT_STATES_HPP
#define CONVERT_TO_JOINT_STATES_HPP

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_InverseKinematics.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_Jacobian.hpp"

// IK, JacobianのPluginをIncludeする必要がある。

#include "Eigen/Dense"

namespace convert_to_joint_states
{
  pluginlib::ClassLoader<control_plugin_base::InverseKinematics> ik_loader("robot_manager", "control_plugin_base::InverseKinematics");
  pluginlib::ClassLoader<control_plugin_base::Jacobian> jac_loader("robot_manager", "control_plugin_base::Jacobian");

  class Default_ConvertToJointStates : public control_plugin_base::ConvertToJointStates 
  {
    public:
      Default_ConvertToJointStates();
      ~Default_ConvertToJointStates(){}

      std::unique_ptr<control_plugin_base::LegJointStatesPattern> convert_into_joint_states(
        const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr
      ) override;
      
    private:
      std::shared_ptr<control_plugin_base::InverseKinematics> ik_;
      std::shared_ptr<control_plugin_base::Jacobian> jac_;

      std::shared_ptr<control_plugin_base::LegStates_ToIK> legL_states_ik_ptr_ = std::make_shared<control_plugin_base::LegStates_ToIK>();
      std::shared_ptr<control_plugin_base::LegStates_ToIK> legR_states_ik_ptr_ = std::make_shared<control_plugin_base::LegStates_ToIK>();
      std::shared_ptr<control_plugin_base::LegStates_ToJac> legL_states_jac_ptr_ = std::make_shared<control_plugin_base::LegStates_ToJac>();
      std::shared_ptr<control_plugin_base::LegStates_ToJac> legR_states_jac_ptr_ = std::make_shared<control_plugin_base::LegStates_ToJac>();

      std::array<Eigen::Vector3d, 6> UnitVec_legL_;
      std::array<Eigen::Vector3d, 6> UnitVec_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_waist_standard_;
      std::array<Eigen::Vector3d, 7> P_legR_waist_standard_;
      Eigen::Matrix3d end_eff_rot;
  };
}

#endif