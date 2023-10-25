#include "convert_to_joint_states/ConvertToJointStates.hpp"

/* MEMO
- 歩行の開始・終了、支持脚・遊脚の判定を、ZMPの位置の値に対して行っている。
  - これでは、直進以外の歩行動作に対応できない。
  - 拡張するか、判定方法を変更する必要がある。
*/

namespace convert_to_joint_states
{
  std::unique_ptr<control_plugin_base::LegJointStatesPattern> Default_ConvertToJointStates::convert_into_joint_states(
    const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr,
    const std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr,
    uint32_t walking_step,
    uint32_t control_step,
    float t
  ) {
    std::unique_ptr<control_plugin_base::LegJointStatesPattern> leg_joint_states_pat_ptr = std::make_unique<control_plugin_base::LegJointStatesPattern>();

    // DEBUG: 着地位置の基準を修正
      // TODO: こいつも他で実行するべき。引数に入ってくる段階で修正されているべき。
    double init_y = foot_step_ptr->foot_pos[0][1];
    std::vector<std::array<double, 2>> foot_pos = foot_step_ptr->foot_pos;
    for(u_int32_t step = 0; step < foot_step_ptr->foot_pos.size(); step++) {
      foot_pos[step][1] -= init_y;
    }

//=====足の軌道計算
    // 遊脚軌道（正弦波）の計算
      // TODO: 両脚支持期間は支持脚切替時に重心速度が急激に変化しないようにするために設けるものである。
    if(t >= T_dsup_/2 && t <= T_sup_-T_dsup_/2) {  // 片足支持期
      old_swing_trajectory_ = swing_trajectory_;
      swing_trajectory_ = height_leg_lift_ * std::sin((3.141592/(T_sup_-T_dsup_))*(t-T_dsup_/2));  
      vel_swing_trajectory_ = ((swing_trajectory_ - old_swing_trajectory_) / control_cycle_);
    }
    else {  // 両脚支持期
      swing_trajectory_ = 0.0;
      old_swing_trajectory_ = 0.0;
      vel_swing_trajectory_ = 0.0;
    }

    // LOG: 遊脚軌道に関するlogの取得
    // WPG_log_SwingTrajectory << swing_trajectory_ << " " << old_swing_trajectory_ << " " << (swing_trajectory_-old_swing_trajectory_) << std::endl;      

    if(foot_pos[walking_step][1] == 0) {  // 歩行開始時、終了時
      int ref_ws; 
      if(walking_step == 0) {  // 歩行開始時
        ref_ws = walking_step+1;
      }
      else {  // 開始時以外
        ref_ws = walking_step-1;
      }
      if(foot_pos[ref_ws][1] >= 0) {  // 左脚支持
        Foot_3D_Pos_ = {  // 左足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_
        };
        Foot_3D_Pos_Swing_ = {  // 右足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_
        };
      }
      else if(foot_pos[ref_ws][1] < 0) {  // 右脚支持
        Foot_3D_Pos_ = {  // 右足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_
        };
        Foot_3D_Pos_Swing_ = {  // 左足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_
        };
      }
    }
    else if(foot_pos[walking_step-1][1] == 0) {  // 歩行開始から1step後
      // 支持脚
      Foot_3D_Pos_ = {  
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y 
        -length_leg_  // z 
      };
      // 遊脚
      if(t <= T_dsup_/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg_
        };
      }
      else if(t >= T_sup_-T_dsup_/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg_
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing_ = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup_/2)/(T_sup_-T_dsup_))),
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_ + swing_trajectory_ // z (遊脚軌道をzから引く) 
        };
      }

    }
    else if(foot_pos[walking_step+1][1] == 0) {  // 歩行終了から1step前
      // 支持脚
      Foot_3D_Pos_ = {
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y  
        -length_leg_  // z 
      };
      // 遊脚
      if(t <= T_dsup_/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],  
          -length_leg_
        };
      }
      else if(t >= T_sup_-T_dsup_/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg_
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing_ = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup_/2)/(T_sup_-T_dsup_)))-(walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]), 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_ + swing_trajectory_
        };
      }
    }
    else {  // 歩行中
      // 支持脚
      Foot_3D_Pos_ = {
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y
        -length_leg_  // z 
      };
      // 遊脚
      if(t <= T_dsup_/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg_
        };
      }
      else if(t >= T_sup_-T_dsup_/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg_
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing_ = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup_/2)/(T_sup_-T_dsup_)))-((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]) / 2),  
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg_ + swing_trajectory_ 
        };
      }
    }

    // LOG: 足軌道のLogの吐き出し
    // WPG_log_FootTrajectory << walking_stabilization_ptr->cog_pos_fix[control_step][0] << " " << walking_stabilization_ptr->cog_pos_fix[control_step][1] << " " << Foot_3D_Pos.transpose() << " " << Foot_3D_Pos_Swing_.transpose() << std::endl;

    // ３次元重心速度の定義
    CoG_3D_Vel_ = {  // 支持脚用
      walking_stabilization_ptr->cog_vel_fix[control_step][0],  // liner x
      walking_stabilization_ptr->cog_vel_fix[control_step][1],  // liner y
      0,  // liner z
      0,  // rotation x
      0,  // rotation y
      0   // rotation z
    };
    CoG_3D_Vel_Swing_ = {  // 遊脚用
      walking_stabilization_ptr->cog_vel_fix[control_step][0],
      walking_stabilization_ptr->cog_vel_fix[control_step][1],
      vel_swing_trajectory_,
      0,
      0,
      0
    };

    // LOG: 重心速度のLog吐き出し
    // WPG_log_SwingTrajectory_Vel << CoG_3D_Vel.transpose() << " " << CoG_3D_Vel_Swing_.transpose() << std::endl;

//=====関節角度・角速度の算出 
    if(foot_pos[walking_step][1] == 0) {  // 歩行開始、終了時
      int ref_ws; 
      if(walking_step == 0) {  // 歩行開始時
        ref_ws = walking_step+1;
      }
      else {  // 歩行終了時
        ref_ws = walking_step-1;
      }
      if(foot_pos[ref_ws][1]-foot_pos[ref_ws+1][1] >= 0) {  // 左脚支持
        // IK
        legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing_;
        ik_->inverse_kinematics(
          legR_states_ik_ptr_,  // 引数
          Q_legR_  // 返り値の参照渡し
        );
        legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_;
        ik_->inverse_kinematics(
          legL_states_ik_ptr_,
          Q_legL_
        );
      }
      if(foot_pos[ref_ws][1]-foot_pos[ref_ws+1][1] < 0) {  // 右脚支持
        legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_;
        ik_->inverse_kinematics(  
          legR_states_ik_ptr_,
          Q_legR_
        );
        legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing_;
        ik_->inverse_kinematics(
          legL_states_ik_ptr_,
          Q_legL_
        );
      }

      // LOG:
      // WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL_, P_legL_waist_standard_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR_;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR_);
      legL_states_jac_ptr_->joint_ang = Q_legL_;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL_);

      // 各関節速度の計算
      if((t >= T_dsup_/2 && t < T_dsup_/2+0.05) || (t > (T_sup_ - T_dsup_/2-0.05) && t <= (T_sup_ - T_dsup_/2))) {
        jointVel_legR_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {
        jointVel_legR_ = Jacobi_legR_.inverse()*CoG_3D_Vel_;
        jointVel_legL_ = Jacobi_legL_.inverse()*CoG_3D_Vel_;
      }

      // 歩行パラメータの代入
      leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR_;
      leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR_[0], jointVel_legR_[1], jointVel_legR_[2], jointVel_legR_[3], jointVel_legR_[4], jointVel_legR_[5]};  // eigen::vectorをstd::arrayに変換するためにこうしている。
      leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL_;
      leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL_[0], jointVel_legL_[1], jointVel_legL_[2], jointVel_legL_[3], jointVel_legL_[4], jointVel_legL_[5]};
    }
    // 左脚支持期
    else if(foot_pos[walking_step][1] > 0) {
      // IK
      legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing_;
      ik_->inverse_kinematics(
        legR_states_ik_ptr_,  // 引数
        Q_legR_  // 返り値の参照渡し
      );
      legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_;
      ik_->inverse_kinematics(
        legL_states_ik_ptr_,
        Q_legL_
      );

      // LOG:
      // WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL_, P_legL_waist_standard_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR_;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR_);
      legL_states_jac_ptr_->joint_ang = Q_legL_;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL_);

      // 各関節速度の計算
      if((t >= T_dsup_/2 && t < T_dsup_/2+0.05) || (t > (T_sup_ - T_dsup_/2-0.05) && t <= (T_sup_ - T_dsup_/2))) {
        jointVel_legR_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {
        jointVel_legR_ = Jacobi_legR_.inverse()*CoG_3D_Vel_Swing_;
        jointVel_legL_ = Jacobi_legL_.inverse()*CoG_3D_Vel_;
      }

      // 歩行パラメータの代入
      leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR_;  // 遊脚
      leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR_[0], jointVel_legR_[1], jointVel_legR_[2], jointVel_legR_[3], jointVel_legR_[4], jointVel_legR_[5]};
      leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL_;  // 支持脚
      leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL_[0], jointVel_legL_[1], jointVel_legL_[2], jointVel_legL_[3], jointVel_legL_[4], jointVel_legL_[5]};
    }
    // 右脚支持期
    else if(foot_pos[walking_step][1] < 0) {
      // IK
      legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_;
      ik_->inverse_kinematics(  
        legR_states_ik_ptr_,
        Q_legR_
      );
      legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing_;
      ik_->inverse_kinematics(
        legL_states_ik_ptr_,
        Q_legL_
      );

      // LOG:
      //WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL_, P_legL_waist_standard_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR_;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR_);
      legL_states_jac_ptr_->joint_ang = Q_legL_;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL_);

      // 各関節速度の計算
      if((t >= T_dsup_/2 && t < T_dsup_/2+0.05) || (t > (T_sup_ - T_dsup_/2-0.05) && t <= (T_sup_ - T_dsup_/2))) {
        jointVel_legR_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {
        jointVel_legR_ = Jacobi_legR_.inverse()*CoG_3D_Vel_;
        jointVel_legL_ = Jacobi_legL_.inverse()*CoG_3D_Vel_Swing_;
      }

      // 歩行パラメータの代入
      leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR_;  // 支持脚
      leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR_[0], jointVel_legR_[1], jointVel_legR_[2], jointVel_legR_[3], jointVel_legR_[4], jointVel_legR_[5]};
      leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL_;  // 遊脚
      leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL_[0], jointVel_legL_[1], jointVel_legL_[2], jointVel_legL_[3], jointVel_legL_[4], jointVel_legL_[5]};
    }

    // std::cout << "Here is default convert_to_joint_states plugin." << std::endl;

    return leg_joint_states_pat_ptr;
  }

  Default_ConvertToJointStates::Default_ConvertToJointStates() {
    // DEBUG
    // WPG_log_FootTrajectory.open(WPG_log_FootTrajectory_path, std::ios::out);
    // WPG_log_SwingTrajectory.open(WPG_WPG_log_SwingTrajectory_path, std::ios::out);
    
    ik_ = ik_loader.createSharedInstance("kinematics::Default_InverseKinematics");
    jac_ = jac_loader.createSharedInstance("kinematics::Default_Jacobian");

    // TODO: Parameterから得たい。それか引数で得たい。
    UnitVec_legL_ = {
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(1, 0, 0)
    };
    UnitVec_legR_ = {
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(1, 0, 0)
    };
    // 脚の関節位置。基準を腰（ｚ軸が股関節と等しい）に修正
    P_legR_waist_standard_ = {  // 右脚
      Eigen::Vector3d(-0.005, -0.037, 0),  // o(基準) -> 1
      Eigen::Vector3d(0, 0, 0),  // 1 -> 2
      Eigen::Vector3d(0, 0, 0),  // 2 -> 3
      Eigen::Vector3d(0, 0, -0.093),  // 3 -> 4
      Eigen::Vector3d(0, 0, -0.093),  // 4 -> 5
      Eigen::Vector3d(0, 0, 0),  // 5 -> 6
      Eigen::Vector3d(0, 0, 0)  // 6 -> a(足裏)
    };
    P_legL_waist_standard_ = {  // 左脚
      Eigen::Vector3d(-0.005, 0.037, 0),
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, -0.093),
      Eigen::Vector3d(0, 0, -0.093),
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0, 0, 0)
    };
    end_eff_rot_ = Eigen::Matrix3d::Identity();

    legL_states_ik_ptr_->end_eff_rot = end_eff_rot_;
    legR_states_ik_ptr_->end_eff_rot = end_eff_rot_;
    legL_states_ik_ptr_->link_len = P_legL_waist_standard_;
    legR_states_ik_ptr_->link_len = P_legR_waist_standard_;

    legL_states_jac_ptr_->link_len = P_legL_waist_standard_;
    legR_states_jac_ptr_->link_len = P_legR_waist_standard_;
    legL_states_jac_ptr_->unit_vec = UnitVec_legL_;
    legR_states_jac_ptr_->unit_vec = UnitVec_legR_;

    // robot
    length_leg_ = 171.856 / 1000;

    // control cycle
    control_cycle_ = 0.01;

    // 時間
    T_sup_ = 0.8;  // 歩行周期
    T_dsup_ = 0.5;  // 両脚支持期間

    // 遊脚軌道関連
    height_leg_lift_ = 0.025;  // 足上げ高さ [m]
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(convert_to_joint_states::Default_ConvertToJointStates, control_plugin_base::ConvertToJointStates)
