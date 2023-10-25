#ifndef CONVERT_TO_JOINT_STATES_HPP
#define CONVERT_TO_JOINT_STATES_HPP

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_InverseKinematics.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_Jacobian.hpp"

#include "Eigen/Dense"

#include <fstream>

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
        const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr,
        const std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr,
        uint32_t walking_step,
        uint32_t control_step,
        float t
      ) override;
      
    private:
      // std::unique_ptr<control_plugin_base::LegJointStatesPattern> leg_joint_states_pat_ptr_ = std::make_unique<control_plugin_base::LegJointStatesPattern>();

      std::shared_ptr<control_plugin_base::InverseKinematics> ik_;
      std::shared_ptr<control_plugin_base::Jacobian> jac_;

      std::shared_ptr<control_plugin_base::LegStates_ToIK> legL_states_ik_ptr_ = std::make_shared<control_plugin_base::LegStates_ToIK>();
      std::shared_ptr<control_plugin_base::LegStates_ToIK> legR_states_ik_ptr_ = std::make_shared<control_plugin_base::LegStates_ToIK>();
      std::shared_ptr<control_plugin_base::LegStates_ToJac> legL_states_jac_ptr_ = std::make_shared<control_plugin_base::LegStates_ToJac>();
      std::shared_ptr<control_plugin_base::LegStates_ToJac> legR_states_jac_ptr_ = std::make_shared<control_plugin_base::LegStates_ToJac>();

      // parameters
      // robot
      std::array<Eigen::Vector3d, 6> UnitVec_legL_;
      std::array<Eigen::Vector3d, 6> UnitVec_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_waist_standard_;
      std::array<Eigen::Vector3d, 7> P_legR_waist_standard_;
      Eigen::Matrix3d end_eff_rot_;
      float length_leg_ = 0;

      // time
      float control_cycle_ = 0;
      float T_sup_ = 0;
      float T_dsup_ = 0;

      // std::ofstream WPG_log_FootTrajectory;
      // std::string WPG_log_FootTrajectory_path = "src/Log/WPG_log_FootTrajectory.dat";
      // std::ofstream WPG_log_SwingTrajectory;
      // std::string WPG_WPG_log_SwingTrajectory_path = "src/Log/WPG_log_SwingTrajectory.dat";

      // trajectory
      double swing_trajectory_ = 0.0;  // 遊脚軌道の値を記録
      double old_swing_trajectory_ = 0.0;  // 微分用
      double vel_swing_trajectory_ = 0.0;  // 遊脚軌道の速度
      float height_leg_lift_ = 0;  // 遊脚軌道の足上げ高さ

      // IKと歩行パラメータの定義・遊脚軌道の反映
      Eigen::Vector<double, 3> Foot_3D_Pos_;
      Eigen::Vector<double, 3> Foot_3D_Pos_Swing_;
      Eigen::Vector<double, 6> CoG_3D_Vel_;
      Eigen::Vector<double, 6> CoG_3D_Vel_Swing_;
      std::array<double, 6> Q_legR_{0, 0, 0, 0, 0, 0};
      std::array<double, 6> Q_legL_{0, 0, 0, 0, 0, 0};
      Eigen::Vector<double, 6> jointVel_legR_;
      Eigen::Vector<double, 6> jointVel_legL_;
      Eigen::Matrix<double, 6, 6> Jacobi_legR_ = Eigen::MatrixXd::Zero(6, 6);
      Eigen::Matrix<double, 6, 6> Jacobi_legL_ = Eigen::MatrixXd::Zero(6, 6);
  };
}

#endif