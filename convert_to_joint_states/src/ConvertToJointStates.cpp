#include "convert_to_joint_states/ConvertToJointStates.hpp"

/* MEMO
- 歩行の開始・終了、支持脚・遊脚の判定を、ZMPの位置の値に対して行っている。
  - これでは、直進以外の歩行動作に対応できない。
  - 拡張するか、判定方法を変更する必要がある。
*/

namespace convert_to_joint_states
{
  Eigen::Vector3d Default_ConvertToJointStates::Convert_strVec2eigenVec(const std::string str_vec) {
    std::stringstream strstream{str_vec};
    std::string buffer;
    std::vector<double> std_vec;
    while(std::getline(strstream, buffer, ' ')) {
      std_vec.push_back(std::stod(buffer));
    }
    // std::cout << std_vec[0] << " " << std_vec[1] << " " << std_vec[2] << std::endl;
    Eigen::Vector3d eigen_vec = {std_vec[0], std_vec[1], std_vec[2]};
    return eigen_vec;
  }
  Eigen::Vector3d Default_ConvertToJointStates::Convert_strVec2eigenVec_UnitVec(const std::string str_vec, std::vector<int>* direction_) {
    std::stringstream strstream{str_vec};
    std::string buffer;
    std::vector<double> std_vec;
    while(std::getline(strstream, buffer, ' ')) {
      std_vec.push_back(std::stod(buffer));
      if(std_vec.back() != 0) {
        direction_->push_back((std_vec.back() > 0) - (std_vec.back() < 0));  // sign
        // std::cout << direction_->back() << " " << direction_->size() << std::endl;
        std_vec.back() = std::abs(std_vec.back());
      }
    }
    // std::cout << std_vec[0] << " " << std_vec[1] << " " << std_vec[2] << std::endl;
    Eigen::Vector3d eigen_vec = {std_vec[0], std_vec[1], std_vec[2]};
    return eigen_vec;
  }

  std::unique_ptr<control_plugin_base::LegJointStatesPattern> Default_ConvertToJointStates::convert_to_joint_states(
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
    if(t >= BOTH_LEG_SUPPORT_PERIOD_/2 && t <= WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_/2) {  // 片足支持期
      old_swing_trajectory_ = swing_trajectory_;
      swing_trajectory_ = HEIGHT_LEG_LIFT_ * std::sin((3.141592/(WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_))*(t-BOTH_LEG_SUPPORT_PERIOD_/2));  
      vel_swing_trajectory_ = ((swing_trajectory_ - old_swing_trajectory_) / CONTROL_CYCLE_);
    }
    else {  // 両脚支持期
      swing_trajectory_ = 0.0;
      old_swing_trajectory_ = 0.0;
      vel_swing_trajectory_ = 0.0;
    }

    // LOG: 遊脚軌道に関するlogの取得
    // WPG_log_SwingTrajectory << swing_trajectory_ << " " << old_swing_trajectory_ << " " << (swing_trajectory_-old_swing_trajectory_) << std::endl;      

    // TODO: 各足先位置の計算、もっと単純化できると思うんだ。
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
          -WAIST_HEIGHT_
        };
        Foot_3D_Pos_Swing_ = {  // 右足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -WAIST_HEIGHT_
        };
      }
      else if(foot_pos[ref_ws][1] < 0) {  // 右脚支持
        Foot_3D_Pos_ = {  // 右足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -WAIST_HEIGHT_
        };
        Foot_3D_Pos_Swing_ = {  // 左足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -WAIST_HEIGHT_
        };
      }
    }
    else if(foot_pos[walking_step-1][1] == 0) {  // 歩行開始から1step後
      // 支持脚
      Foot_3D_Pos_ = {  
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y 
        -WAIST_HEIGHT_  // z 
      };
      // 遊脚
      if(t <= BOTH_LEG_SUPPORT_PERIOD_/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -WAIST_HEIGHT_
        };
      }
      else if(t >= WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -WAIST_HEIGHT_
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing_ = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-BOTH_LEG_SUPPORT_PERIOD_/2)/(WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_))),
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -WAIST_HEIGHT_ + swing_trajectory_ // z (遊脚軌道をzから引く) 
        };
      }

    }
    else if(foot_pos[walking_step+1][1] == 0) {  // 歩行終了から1step前
      // 支持脚
      Foot_3D_Pos_ = {
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y  
        -WAIST_HEIGHT_  // z 
      };
      // 遊脚
      if(t <= BOTH_LEG_SUPPORT_PERIOD_/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],  
          -WAIST_HEIGHT_
        };
      }
      else if(t >= WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -WAIST_HEIGHT_
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing_ = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-BOTH_LEG_SUPPORT_PERIOD_/2)/(WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_)))-(walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]), 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -WAIST_HEIGHT_ + swing_trajectory_
        };
      }
    }
    else {  // 歩行中
      // 支持脚
      Foot_3D_Pos_ = {
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y
        -WAIST_HEIGHT_  // z 
      };
      // 遊脚
      if(t <= BOTH_LEG_SUPPORT_PERIOD_/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -WAIST_HEIGHT_
        };
      }
      else if(t >= WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing_ = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -WAIST_HEIGHT_
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing_ = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-BOTH_LEG_SUPPORT_PERIOD_/2)/(WALKING_CYCLE_-BOTH_LEG_SUPPORT_PERIOD_)))-((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]) / 2),  
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(WALKING_CYCLE_)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -WAIST_HEIGHT_ + swing_trajectory_ 
        };
      }
    }

    // LOG: 足軌道のLogの吐き出し
    // WPG_log_FootTrajectory << walking_stabilization_ptr->cog_pos_fix[control_step][0] << " " << walking_stabilization_ptr->cog_pos_fix[control_step][1] << " " << Foot_3D_Pos.transpose() << " " << Foot_3D_Pos_Swing_.transpose() << std::endl;

    // ３次元重心速度の定義
      // ロボットのローカル座標から見た両脚の足先速度（X, Y）は、ロボットの重心速度（X, Y）と等しい。
    CoG_3D_Vel_ = {  // 支持脚用
      walking_stabilization_ptr->cog_vel_fix[control_step][0],  // liner x
      walking_stabilization_ptr->cog_vel_fix[control_step][1],  // liner y
      0,  // liner z
      0,  // rotation x
      0,  // rotation y
      0   // rotation z
    };
    CoG_3D_Vel_Swing_ = {  // 遊脚用
      walking_stabilization_ptr->cog_vel_fix[control_step][0],  // 
      walking_stabilization_ptr->cog_vel_fix[control_step][1],  // 
      vel_swing_trajectory_,
      0,
      0,
      0
    };

    // LOG: 重心速度のLog吐き出し
    // WPG_log_SwingTrajectory_Vel << CoG_3D_Vel.transpose() << " " << CoG_3D_Vel_Swing_.transpose() << std::endl;

//=====関節角度・角速度の算出 
    if(foot_pos[walking_step][1] == 0) {  // 歩行開始、終了時 (脚は踏み出さず、重心移動のみ)
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
      // WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, RIGHT_LEG_LINK_LENGTH_, 6).transpose() << " " << FK_.getFK(Q_legL_, LEFT_LEG_LINK_LENGTH_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR_;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR_);
      legL_states_jac_ptr_->joint_ang = Q_legL_;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL_);

      // 各関節速度の計算
      if((t >= BOTH_LEG_SUPPORT_PERIOD_/2 && t < BOTH_LEG_SUPPORT_PERIOD_/2+0.05) || (t > (WALKING_CYCLE_ - BOTH_LEG_SUPPORT_PERIOD_/2-0.05) && t <= (WALKING_CYCLE_ - BOTH_LEG_SUPPORT_PERIOD_/2))) {
        jointVel_legR_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {  
          jointVel_legR_ = Jacobi_legR_.inverse()*CoG_3D_Vel_;
          jointVel_legL_ = Jacobi_legL_.inverse()*CoG_3D_Vel_; 
      }

      // // 歩行パラメータの代入
      // leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR_;
      // leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR_[0], jointVel_legR_[1], jointVel_legR_[2], jointVel_legR_[3], jointVel_legR_[4], jointVel_legR_[5]};  // eigen::vectorをstd::arrayに変換するためにこうしている。
      // leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL_;
      // leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL_[0], jointVel_legL_[1], jointVel_legL_[2], jointVel_legL_[3], jointVel_legL_[4], jointVel_legL_[5]};
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
      // WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, RIGHT_LEG_LINK_LENGTH_, 6).transpose() << " " << FK_.getFK(Q_legL_, LEFT_LEG_LINK_LENGTH_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR_;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR_);
      legL_states_jac_ptr_->joint_ang = Q_legL_;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL_);

      // 各関節速度の計算
      if((t >= BOTH_LEG_SUPPORT_PERIOD_/2 && t < BOTH_LEG_SUPPORT_PERIOD_/2+0.05) || (t > (WALKING_CYCLE_ - BOTH_LEG_SUPPORT_PERIOD_/2-0.05) && t <= (WALKING_CYCLE_ - BOTH_LEG_SUPPORT_PERIOD_/2))) {
        jointVel_legR_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {  
        jointVel_legR_ = Jacobi_legR_.inverse()*CoG_3D_Vel_Swing_;
        jointVel_legL_ = Jacobi_legL_.inverse()*CoG_3D_Vel_;
      }

      // // 歩行パラメータの代入
      // leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR_;  // 遊脚
      // leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR_[0], jointVel_legR_[1], jointVel_legR_[2], jointVel_legR_[3], jointVel_legR_[4], jointVel_legR_[5]};
      // leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL_;  // 支持脚
      // leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL_[0], jointVel_legL_[1], jointVel_legL_[2], jointVel_legL_[3], jointVel_legL_[4], jointVel_legL_[5]};
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
      //WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, RIGHT_LEG_LINK_LENGTH_, 6).transpose() << " " << FK_.getFK(Q_legL_, LEFT_LEG_LINK_LENGTH_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR_;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR_);
      legL_states_jac_ptr_->joint_ang = Q_legL_;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL_);

      // 各関節速度の計算
      if((t >= BOTH_LEG_SUPPORT_PERIOD_/2 && t < BOTH_LEG_SUPPORT_PERIOD_/2+0.05) || (t > (WALKING_CYCLE_ - BOTH_LEG_SUPPORT_PERIOD_/2-0.05) && t <= (WALKING_CYCLE_ - BOTH_LEG_SUPPORT_PERIOD_/2))) {
        jointVel_legR_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL_ = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {  
        jointVel_legR_ = Jacobi_legR_.inverse()*CoG_3D_Vel_;
        jointVel_legL_ = Jacobi_legL_.inverse()*CoG_3D_Vel_Swing_;
      }
    }

    // 歩行パラメータの代入
    leg_joint_states_pat_ptr->joint_ang_pat_legR = {Q_legR_[0]*right_leg_unit_vector_direction_[0], 
                                                    Q_legR_[1]*right_leg_unit_vector_direction_[1], 
                                                    Q_legR_[2]*right_leg_unit_vector_direction_[2], 
                                                    Q_legR_[3]*right_leg_unit_vector_direction_[3], 
                                                    Q_legR_[4]*right_leg_unit_vector_direction_[4], 
                                                    Q_legR_[5]*right_leg_unit_vector_direction_[5]};
    leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR_[0], jointVel_legR_[1], jointVel_legR_[2], jointVel_legR_[3], jointVel_legR_[4], jointVel_legR_[5]};
    leg_joint_states_pat_ptr->joint_ang_pat_legL = {Q_legL_[0]*left_leg_unit_vector_direction_[0], 
                                                    Q_legL_[1]*left_leg_unit_vector_direction_[1], 
                                                    Q_legL_[2]*left_leg_unit_vector_direction_[2], 
                                                    Q_legL_[3]*left_leg_unit_vector_direction_[3], 
                                                    Q_legL_[4]*left_leg_unit_vector_direction_[4], 
                                                    Q_legL_[5]*left_leg_unit_vector_direction_[5]};
    leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL_[0], jointVel_legL_[1], jointVel_legL_[2], jointVel_legL_[3], jointVel_legL_[4], jointVel_legL_[5]};

    // std::cout << "Here is default convert_to_joint_states plugin." << std::endl;

    return leg_joint_states_pat_ptr;
  }

  Default_ConvertToJointStates::Default_ConvertToJointStates() {
    // DEBUG
    // WPG_log_FootTrajectory.open(WPG_log_FootTrajectory_path, std::ios::out);
    // WPG_log_SwingTrajectory.open(WPG_WPG_log_SwingTrajectory_path, std::ios::out);
    
    ik_ = ik_loader.createSharedInstance("kinematics::Default_InverseKinematics");
    jac_ = jac_loader.createSharedInstance("kinematics::Default_Jacobian");

    // client parrameters
    node_ptr_ = rclcpp::Node::make_shared("ConvertToJointStates");
    client_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "RobotParameterServer");
    
    // param: description
    ROBOT_NAME_ = client_param_->get_parameter<std::string>("robot_description.robot_name");
    
    // param: control
    WAIST_HEIGHT_ = client_param_->get_parameter<double>("control_constant.waist_pos_z");
    HEIGHT_LEG_LIFT_ = client_param_->get_parameter<double>("control_constant.height_leg_lift");
    WALKING_CYCLE_ = client_param_->get_parameter<double>("control_times.walking_cycle");
    CONTROL_CYCLE_ = client_param_->get_parameter<double>("control_times.control_cycle");
    BOTH_LEG_SUPPORT_PERIOD_ = client_param_->get_parameter<double>("control_times.both_leg_support_period");
    SINGLE_LEG_SUPPORT_PERIOD_ = client_param_->get_parameter<double>("control_times.single_leg_support_period");
    
    //param: limb
    LEFT_LEG_NAME_ = client_param_->get_parameter<std::string>(ROBOT_NAME_+"_limb.limb_names.left_leg");
    RIGHT_LEG_NAME_ = client_param_->get_parameter<std::string>(ROBOT_NAME_+"_limb.limb_names.right_leg");
    LEFT_LEG_UNIT_VECTOR_STRING_ = client_param_->get_parameter<std::vector<std::string>>(ROBOT_NAME_+"_limb.limb_without_fixed_joints."+LEFT_LEG_NAME_+".joint_unit_vector");
    RIGHT_LEG_UNIT_VECTOR_STRING_ = client_param_->get_parameter<std::vector<std::string>>(ROBOT_NAME_+"_limb.limb_without_fixed_joints."+RIGHT_LEG_NAME_+".joint_unit_vector");
    LEFT_LEG_LINK_LENGTH_STRING_ = client_param_->get_parameter<std::vector<std::string>>(ROBOT_NAME_+"_limb.limb_without_fixed_joints."+LEFT_LEG_NAME_+".link_length");
    RIGHT_LEG_LINK_LENGTH_STRING_ = client_param_->get_parameter<std::vector<std::string>>(ROBOT_NAME_+"_limb.limb_without_fixed_joints."+RIGHT_LEG_NAME_+".link_length");
    
    LEFT_LEG_UNIT_VECTOR_ = {
      Convert_strVec2eigenVec_UnitVec(LEFT_LEG_UNIT_VECTOR_STRING_[0], &left_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(LEFT_LEG_UNIT_VECTOR_STRING_[1], &left_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(LEFT_LEG_UNIT_VECTOR_STRING_[2], &left_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(LEFT_LEG_UNIT_VECTOR_STRING_[3], &left_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(LEFT_LEG_UNIT_VECTOR_STRING_[4], &left_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(LEFT_LEG_UNIT_VECTOR_STRING_[5], &left_leg_unit_vector_direction_)
    };
    RIGHT_LEG_UNIT_VECTOR_ = {
      Convert_strVec2eigenVec_UnitVec(RIGHT_LEG_UNIT_VECTOR_STRING_[0], &right_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(RIGHT_LEG_UNIT_VECTOR_STRING_[1], &right_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(RIGHT_LEG_UNIT_VECTOR_STRING_[2], &right_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(RIGHT_LEG_UNIT_VECTOR_STRING_[3], &right_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(RIGHT_LEG_UNIT_VECTOR_STRING_[4], &right_leg_unit_vector_direction_),
      Convert_strVec2eigenVec_UnitVec(RIGHT_LEG_UNIT_VECTOR_STRING_[5], &right_leg_unit_vector_direction_)
    };
    LEFT_LEG_LINK_LENGTH_ = {
      Convert_strVec2eigenVec(LEFT_LEG_LINK_LENGTH_STRING_[0]),
      Convert_strVec2eigenVec(LEFT_LEG_LINK_LENGTH_STRING_[1]),
      Convert_strVec2eigenVec(LEFT_LEG_LINK_LENGTH_STRING_[2]),
      Convert_strVec2eigenVec(LEFT_LEG_LINK_LENGTH_STRING_[3]),
      Convert_strVec2eigenVec(LEFT_LEG_LINK_LENGTH_STRING_[4]),
      Convert_strVec2eigenVec(LEFT_LEG_LINK_LENGTH_STRING_[5])
    };
    LEFT_LEG_LINK_LENGTH_[0][2] = 0;  // 重心位置を腰とする処理。
    LEFT_LEG_LINK_LENGTH_[1][2] = 0;  // DEBUG: ROBOTIS_OP2に合わせた処理。
    LEFT_LEG_LINK_LENGTH_[6] = Eigen::Vector3d(0, 0, 0);  // joint-6から足先末端までのlinkを追加. Forward Kinematicsに合わせる。
    RIGHT_LEG_LINK_LENGTH_ = {
      Convert_strVec2eigenVec(RIGHT_LEG_LINK_LENGTH_STRING_[0]),
      Convert_strVec2eigenVec(RIGHT_LEG_LINK_LENGTH_STRING_[1]),
      Convert_strVec2eigenVec(RIGHT_LEG_LINK_LENGTH_STRING_[2]),
      Convert_strVec2eigenVec(RIGHT_LEG_LINK_LENGTH_STRING_[3]),
      Convert_strVec2eigenVec(RIGHT_LEG_LINK_LENGTH_STRING_[4]),
      Convert_strVec2eigenVec(RIGHT_LEG_LINK_LENGTH_STRING_[5])
    };
    RIGHT_LEG_LINK_LENGTH_[0][2] = 0;
    RIGHT_LEG_LINK_LENGTH_[1][2] = 0;
    RIGHT_LEG_LINK_LENGTH_[6] = Eigen::Vector3d(0, 0, 0);

    // // TODO: Parameterから得たい。それか引数で得たい。
    // LEFT_LEG_UNIT_VECTOR_ = {
    //   Eigen::Vector3d(0, 0, 1),
    //   Eigen::Vector3d(1, 0, 0),
    //   Eigen::Vector3d(0, 1, 0),
    //   Eigen::Vector3d(0, 1, 0),
    //   Eigen::Vector3d(0, 1, 0),
    //   Eigen::Vector3d(1, 0, 0)
    // };
    // RIGHT_LEG_UNIT_VECTOR_ = {
    //   Eigen::Vector3d(0, 0, 1),
    //   Eigen::Vector3d(1, 0, 0),
    //   Eigen::Vector3d(0, 1, 0),
    //   Eigen::Vector3d(0, 1, 0),
    //   Eigen::Vector3d(0, 1, 0),
    //   Eigen::Vector3d(1, 0, 0)
    // };
    // // 脚の関節位置。基準を腰（ｚ軸が股関節と等しい）に修正
    // RIGHT_LEG_LINK_LENGTH_ = {  // 右脚
    //   Eigen::Vector3d(-0.005, -0.037, 0),  // o(基準) -> 1
    //   Eigen::Vector3d(0, 0, 0),  // 1 -> 2
    //   Eigen::Vector3d(0, 0, 0),  // 2 -> 3
    //   Eigen::Vector3d(0, 0, -0.093),  // 3 -> 4
    //   Eigen::Vector3d(0, 0, -0.093),  // 4 -> 5
    //   Eigen::Vector3d(0, 0, 0),  // 5 -> 6
    //   Eigen::Vector3d(0, 0, 0)  // 6 -> a(足裏)
    // };
    // LEFT_LEG_LINK_LENGTH_ = {  // 左脚
    //   Eigen::Vector3d(-0.005, 0.037, 0),
    //   Eigen::Vector3d(0, 0, 0),
    //   Eigen::Vector3d(0, 0, 0),
    //   Eigen::Vector3d(0, 0, -0.093),
    //   Eigen::Vector3d(0, 0, -0.093),
    //   Eigen::Vector3d(0, 0, 0),
    //   Eigen::Vector3d(0, 0, 0)
    // };
    end_eff_rot_ = Eigen::Matrix3d::Identity();

    legL_states_ik_ptr_->end_eff_rot = end_eff_rot_;
    legR_states_ik_ptr_->end_eff_rot = end_eff_rot_;
    legL_states_ik_ptr_->link_len = LEFT_LEG_LINK_LENGTH_;
    legR_states_ik_ptr_->link_len = RIGHT_LEG_LINK_LENGTH_;

    legL_states_jac_ptr_->link_len = LEFT_LEG_LINK_LENGTH_;
    legR_states_jac_ptr_->link_len = RIGHT_LEG_LINK_LENGTH_;
    legL_states_jac_ptr_->unit_vec = LEFT_LEG_UNIT_VECTOR_;
    legR_states_jac_ptr_->unit_vec = RIGHT_LEG_UNIT_VECTOR_;

    // robot
    // WAIST_HEIGHT_ = 171.856 / 1000;

    // control cycle
    // CONTROL_CYCLE_ = 0.01;

    // 時間
    // WALKING_CYCLE_ = 0.8;  // 歩行周期
    // BOTH_LEG_SUPPORT_PERIOD_ = 0.5;  // 両脚支持期間

    // 遊脚軌道関連
    // HEIGHT_LEG_LIFT_ = 0.025;  // 足上げ高さ [m]

    RCLCPP_INFO(node_ptr_->get_logger(), "Start Up ConvertToJointStates.");
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(convert_to_joint_states::Default_ConvertToJointStates, control_plugin_base::ConvertToJointStates)
