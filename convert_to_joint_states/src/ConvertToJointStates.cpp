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
    auto leg_joint_states_pat_ptr = std::make_unique<control_plugin_base::LegJointStatesPattern>();

    // robot
    float length_leg = 171.856 / 1000;

    // control cycle
    float control_cycle = 0.01;

    // 時間
    float T_sup = 0.8;  // 歩行周期
    float T_dsup = 0.5;  // 両脚支持期間

    // 遊脚軌道関連
    float height_leg_lift = 0.025;  // 足上げ高さ [m]

    // IKと歩行パラメータの定義・遊脚軌道の反映
    Eigen::Vector<double, 3> Foot_3D_Pos;
    Eigen::Vector<double, 3> Foot_3D_Pos_Swing;
    Eigen::Vector<double, 6> CoG_3D_Vel;
    Eigen::Vector<double, 6> CoG_3D_Vel_Swing;
    std::array<double, 6> Q_legR{0, 0, 0, 0, 0, 0};
    std::array<double, 6> Q_legL{0, 0, 0, 0, 0, 0};
    Eigen::Vector<double, 6> jointVel_legR;
    Eigen::Vector<double, 6> jointVel_legL;
    Eigen::Matrix<double, 6, 6> Jacobi_legR = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix<double, 6, 6> Jacobi_legL = Eigen::MatrixXd::Zero(6, 6);

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
    if(t >= T_dsup/2 && t <= T_sup-T_dsup/2) {  // 片足支持期
      old_swing_trajectory = swing_trajectory;
      swing_trajectory = height_leg_lift * std::sin((3.141592/(T_sup-T_dsup))*(t-T_dsup/2));  
      vel_swing_trajectory = ((swing_trajectory - old_swing_trajectory) / control_cycle);
    }
    else {  // 両脚支持期
      swing_trajectory = 0.0;
      old_swing_trajectory = 0.0;
      vel_swing_trajectory = 0.0;
    }

    // LOG: 遊脚軌道に関するlogの取得
    // WPG_log_SwingTrajectory << swing_trajectory << " " << old_swing_trajectory << " " << (swing_trajectory-old_swing_trajectory) << std::endl;      

    if(foot_pos[walking_step][1] == 0) {  // 歩行開始時、終了時
      int ref_ws; 
      if(walking_step == 0) {  // 歩行開始時
        ref_ws = walking_step+1;
      }
      else {  // 開始時以外
        ref_ws = walking_step-1;
      }
      if(foot_pos[ref_ws][1] >= 0) {  // 左脚支持
        Foot_3D_Pos = {  // 左足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg
        };
        Foot_3D_Pos_Swing = {  // 右足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg
        };
      }
      else if(foot_pos[ref_ws][1] < 0) {  // 右脚支持
        Foot_3D_Pos = {  // 右足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg
        };
        Foot_3D_Pos_Swing = {  // 左足
          walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg
        };
      }
    }
    else if(foot_pos[walking_step-1][1] == 0) {  // 歩行開始から1step後
      // 支持脚
      Foot_3D_Pos = {  
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y 
        -length_leg  // z 
      };
      // 遊脚
      if(t <= T_dsup/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg
        };
      }
      else if(t >= T_sup-T_dsup/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup/2)/(T_sup-T_dsup))),
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg + swing_trajectory // z (遊脚軌道をzから引く) 
        };
      }

    }
    else if(foot_pos[walking_step+1][1] == 0) {  // 歩行終了から1step前
      // 支持脚
      Foot_3D_Pos = {
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y  
        -length_leg  // z 
      };
      // 遊脚
      if(t <= T_dsup/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],  
          -length_leg
        };
      }
      else if(t >= T_sup-T_dsup/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup/2)/(T_sup-T_dsup)))-(walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]), 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg + swing_trajectory
        };
      }
    }
    else {  // 歩行中
      // 支持脚
      Foot_3D_Pos = {
        walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
        walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y
        -length_leg  // z 
      };
      // 遊脚
      if(t <= T_dsup/2) {  // 両脚支持（前半）
        Foot_3D_Pos_Swing = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg
        };
      }
      else if(t >= T_sup-T_dsup/2) {  // 両脚支持（後半）
        Foot_3D_Pos_Swing = {
          walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
          -length_leg
        };
      }
      else {  // 片脚支持
        Foot_3D_Pos_Swing = {
          ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup/2)/(T_sup-T_dsup)))-((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]) / 2),  
          walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
          -length_leg + swing_trajectory 
        };
      }
    }

    // LOG: 足軌道のLogの吐き出し
    // WPG_log_FootTrajectory << walking_stabilization_ptr->cog_pos_fix[control_step][0] << " " << walking_stabilization_ptr->cog_pos_fix[control_step][1] << " " << Foot_3D_Pos.transpose() << " " << Foot_3D_Pos_Swing.transpose() << std::endl;

    // ３次元重心速度の定義
    CoG_3D_Vel = {  // 支持脚用
      walking_stabilization_ptr->cog_vel_fix[control_step][0],  // liner x
      walking_stabilization_ptr->cog_vel_fix[control_step][1],  // liner y
      0,  // liner z
      0,  // rotation x
      0,  // rotation y
      0   // rotation z
    };
    CoG_3D_Vel_Swing = {  // 遊脚用
      walking_stabilization_ptr->cog_vel_fix[control_step][0],
      walking_stabilization_ptr->cog_vel_fix[control_step][1],
      vel_swing_trajectory,
      0,
      0,
      0
    };

    // LOG: 重心速度のLog吐き出し
    // WPG_log_SwingTrajectory_Vel << CoG_3D_Vel.transpose() << " " << CoG_3D_Vel_Swing.transpose() << std::endl;

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
        legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing;
        ik_->inverse_kinematics(
          legR_states_ik_ptr_,  // 引数
          Q_legR  // 返り値の参照渡し
        );
        legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos;
        ik_->inverse_kinematics(
          legL_states_ik_ptr_,
          Q_legL
        );
      }
      if(foot_pos[ref_ws][1]-foot_pos[ref_ws+1][1] < 0) {  // 右脚支持
        legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos;
        ik_->inverse_kinematics(  
          legR_states_ik_ptr_,
          Q_legR
        );
        legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing;
        ik_->inverse_kinematics(
          legL_states_ik_ptr_,
          Q_legL
        );
      }

      // LOG:
      // WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL, P_legL_waist_standard_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR);
      legL_states_jac_ptr_->joint_ang = Q_legL;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL);

      // 各関節速度の計算
      if((t >= T_dsup/2 && t < T_dsup/2+0.05) || (t > (T_sup - T_dsup/2-0.05) && t <= (T_sup - T_dsup/2))) {
        jointVel_legR = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {
        jointVel_legR = Jacobi_legR.inverse()*CoG_3D_Vel;
        jointVel_legL = Jacobi_legL.inverse()*CoG_3D_Vel;
      }

      // 歩行パラメータの代入
      leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR;
      leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]};  // eigen::vectorをstd::arrayに変換するためにこうしている。
      leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL;
      leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]};
    }
    // 左脚支持期
    else if(foot_pos[walking_step][1] > 0) {
      // IK
      legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing;
      ik_->inverse_kinematics(
        legR_states_ik_ptr_,  // 引数
        Q_legR  // 返り値の参照渡し
      );
      legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos;
      ik_->inverse_kinematics(
        legL_states_ik_ptr_,
        Q_legL
      );

      // LOG:
      // WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL, P_legL_waist_standard_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR);
      legL_states_jac_ptr_->joint_ang = Q_legL;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL);

      // 各関節速度の計算
      if((t >= T_dsup/2 && t < T_dsup/2+0.05) || (t > (T_sup - T_dsup/2-0.05) && t <= (T_sup - T_dsup/2))) {
        jointVel_legR = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {
        jointVel_legR = Jacobi_legR.inverse()*CoG_3D_Vel_Swing;
        jointVel_legL = Jacobi_legL.inverse()*CoG_3D_Vel;
      }

      // 歩行パラメータの代入
      leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR;  // 遊脚
      leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]};
      leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL;  // 支持脚
      leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]};
    }
    // 右脚支持期
    else if(foot_pos[walking_step][1] < 0) {
      // IK
      legR_states_ik_ptr_->end_eff_pos = Foot_3D_Pos;
      ik_->inverse_kinematics(  
        legR_states_ik_ptr_,
        Q_legR
      );
      legL_states_ik_ptr_->end_eff_pos = Foot_3D_Pos_Swing;
      ik_->inverse_kinematics(
        legL_states_ik_ptr_,
        Q_legL
      );

      // LOG:
      //WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL, P_legL_waist_standard_, 6).transpose() << std::endl;

      // Jacobianの計算
      legR_states_jac_ptr_->joint_ang = Q_legR;
      jac_->jacobian(legR_states_jac_ptr_, Jacobi_legR);
      legL_states_jac_ptr_->joint_ang = Q_legL;
      jac_->jacobian(legL_states_jac_ptr_, Jacobi_legL);

      // 各関節速度の計算
      if((t >= T_dsup/2 && t < T_dsup/2+0.05) || (t > (T_sup - T_dsup/2-0.05) && t <= (T_sup - T_dsup/2))) {
        jointVel_legR = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        jointVel_legL = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
      }
      else {
        jointVel_legR = Jacobi_legR.inverse()*CoG_3D_Vel;
        jointVel_legL = Jacobi_legL.inverse()*CoG_3D_Vel_Swing;
      }

      // 歩行パラメータの代入
      leg_joint_states_pat_ptr->joint_ang_pat_legR = Q_legR;  // 支持脚
      leg_joint_states_pat_ptr->joint_vel_pat_legR = {jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]};
      leg_joint_states_pat_ptr->joint_ang_pat_legL = Q_legL;  // 遊脚
      leg_joint_states_pat_ptr->joint_vel_pat_legL = {jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]};
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
    end_eff_rot = Eigen::Matrix3d::Identity();

    legL_states_ik_ptr_->end_eff_rot = end_eff_rot;
    legR_states_ik_ptr_->end_eff_rot = end_eff_rot;
    legL_states_ik_ptr_->link_len = P_legL_waist_standard_;
    legR_states_ik_ptr_->link_len = P_legR_waist_standard_;

    legL_states_jac_ptr_->link_len = P_legL_waist_standard_;
    legR_states_jac_ptr_->link_len = P_legR_waist_standard_;
    legL_states_jac_ptr_->unit_vec = UnitVec_legL_;
    legR_states_jac_ptr_->unit_vec = UnitVec_legR_;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(convert_to_joint_states::Default_ConvertToJointStates, control_plugin_base::ConvertToJointStates)
