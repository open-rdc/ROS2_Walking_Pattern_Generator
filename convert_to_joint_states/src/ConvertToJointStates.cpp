#include "convert_to_joint_states/ConvertToJointStates.hpp"

/* MEMO
- 歩行の開始・終了、支持脚・遊脚の判定を、ZMPの位置の値に対して行っている。
  - これでは、直進以外の歩行動作に対応できない。
  - 拡張するか、判定方法を変更する必要がある。
*/

namespace convert_to_joint_states
{
  std::unique_ptr<control_plugin_base::LegJointStatesPattern> Default_ConvertToJointStates::convert_into_joint_states(
    const std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr
  ) {
    auto leg_joint_states_pat_ptr = std::make_unique<control_plugin_base::LegJointStatesPattern>();
    // leg_joint_states_pat_ptr->joint_ang_pat_legL = {{1, 2, 3, 4, 5, 6}};
    // leg_joint_states_pat_ptr->joint_ang_pat_legR = {{7, 8, 9, 0, 1, 2}};
    // leg_joint_states_pat_ptr->joint_vel_pat_legL = {{3, 4, 5, 6, 7, 8}};
    // leg_joint_states_pat_ptr->joint_vel_pat_legR = {{9, 0, 1, 2, 3, 4}};

/* WalkingPattern Type
  struct WalkingPattern {
    std::vector<std::array<double, 3>> cog_pos;
    std::vector<std::array<double, 3>> cog_vel;
    std::vector<std::array<double, 2>> zmp_pos;
  }; 
*/
/* WalkingStabilization Type
  struct WalkingStabilization {
    std::vector<std::array<double, 3>> cog_pos_fix;
    std::vector<std::array<double, 3>> cog_vel_fix;
    std::vector<std::array<double, 2>> zmp_pos_fix;
  };
*/

    std::cout << "Here is default convert to joint states class." << std::endl;

//==COPY==
    // robot
    float length_leg = 171.856 / 1000;

    // control cycle
    float control_cycle = 0.01;

    // 時間
    float t = 0;
    float T_sup = 0.8;
    float T_dsup = 0.5;
    float walking_time = 0;
    float walking_time_max = 5.6;  // TODO: この、歩行パラメータ最終着地時間[s]は引数で得る必要がある

    // 遊脚軌道関連
    float height_leg_lift = 0.025;  // 足上げ高さ [m]
    double swing_trajectory = 0.0;  // 遊脚軌道の値を記録
    double old_swing_trajectory = 0.0;  // 微分用
    double vel_swing_trajectory = 0.0;  // 遊脚軌道の速度
    int walking_step = 0;
    int control_step = 0;

    // // 位置の基準を修正（Y軸基準を右足裏から胴体中心へ）
    // double Initwalking_stabilization_ptr->zmp_pos_fixy = walking_stabilization_ptr->zmp_pos_fix[0][1];
    // for(u_int step = 0; step < walking_stabilization_ptr->zmp_pos_fix.size(); step++) {
    //   walking_stabilization_ptr->zmp_pos_fix[step][1] -= Initwalking_stabilization_ptr->zmp_pos_fixy;
    //   walking_stabilization_ptr->zmp_pos_fix[step][0] -= Initwalking_stabilization_ptr->zmp_pos_fixy;
    // }

    // IKと歩行パラメータの定義・遊脚軌道の反映
    Eigen::Vector<double, 3> Foot_3D_Pos;
    Eigen::Vector<double, 3> Foot_3D_Pos_Swing;
    Eigen::Vector<double, 6> CoG_3D_Vel;
    Eigen::Vector<double, 6> CoG_3D_Vel_Swing;
    Eigen::Vector<double, 6> jointVel_legR;
    Eigen::Vector<double, 6> jointVel_legL;

    // 各関節角度・角速度を生成
    while(walking_time <= walking_time_max) {

      // 支持脚切替タイミングの判定
      if(t >= T_sup - 0.01) {
        // 支持脚切替のための更新
        t = 0;
        walking_step++;
      }

      // // 位置の基準を修正（Y軸基準を右足裏から胴体中心へ）
      // walking_stabilization_ptr->cog_pos_fix[control_step][1] -= Initwalking_stabilization_ptr->zmp_pos_fixy;

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

//=====足の軌道計算
      if(walking_stabilization_ptr->zmp_pos_fix[walking_step][1] == 0) {  // 歩行開始時、終了時
        int ref_ws; 
        if(walking_step == 0) {  // 歩行開始時
          ref_ws = walking_step+1;
        }
        else {  // 開始時以外
          ref_ws = walking_step-1;
        }
        if(walking_stabilization_ptr->zmp_pos_fix[ref_ws][1] >= 0) {  // 左脚支持
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
        else if(walking_stabilization_ptr->zmp_pos_fix[ref_ws][1] < 0) {  // 右脚支持
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
      else if(walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1] == 0) {  // 歩行開始から1step後
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
      else if(walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1] == 0) {  // 歩行終了から1step前
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
      //WPG_log_FootTrajectory << walking_stabilization_ptr->cog_pos_fix[control_step][0] << " " << walking_stabilization_ptr->cog_pos_fix[control_step][1] << " " << Foot_3D_Pos.transpose() << " " << Foot_3D_Pos_Swing.transpose() << std::endl;

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
      if(walking_stabilization_ptr->zmp_pos_fix[walking_step][1] == 0) {  // 歩行開始、終了時
        int ref_ws; 
        if(walking_step == 0) {  // 歩行開始時
          ref_ws = walking_step+1;
        }
        else {  // 歩行終了時
          ref_ws = walking_step-1;
        }
        if(walking_stabilization_ptr->zmp_pos_fix[ref_ws][1]-walking_stabilization_ptr->zmp_pos_fix[ref_ws+1][1] >= 0) {  // 左脚支持
          // IK
          Q_legR_ = IK_.getIK(  // IKを解いて、各関節角度を取得
            P_legR_waist_standard_,  // 脚の各リンク長
            Foot_3D_Pos_Swing,  // 重心位置を元にした足の位置 
            R_target_leg  // 足の回転行列。床面と並行なので、ただの単位行列。
          );
          Q_legL_ = IK_.getIK(
            P_legL_waist_standard_,
            Foot_3D_Pos,
            R_target_leg
          );
        }
        if(walking_stabilization_ptr->zmp_pos_fix[ref_ws][1]-walking_stabilization_ptr->zmp_pos_fix[ref_ws+1][1] < 0) {  // 右脚支持
          Q_legR_ = IK_.getIK(  
            P_legR_waist_standard_,
            Foot_3D_Pos,   
            R_target_leg  
          );
          Q_legL_ = IK_.getIK(
            P_legL_waist_standard_,
            Foot_3D_Pos_Swing,
            R_target_leg
          );
        }

        // LOG:
        //WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL_, P_legL_waist_standard_, 6).transpose() << std::endl;

        // Jacobianの計算、Jacobianを記憶するクラス変数の更新
        // JacobiMatrix_leg(Q_legR_, Q_legL_);
        // Jacobi_legR_ = JacobiMatrix_leg(Q_legR_, UnitVec_legR_, P_legR_waist_standard_);
        // Jacobi_legL_ = JacobiMatrix_leg(Q_legL_, UnitVec_legL_, P_legL_waist_standard_);
        Jacobi_legR_ = Jacobian_.JacobiMatrix_leg(Q_legR_, UnitVec_legR_, P_legR_waist_standard_);
        Jacobi_legL_ = Jacobian_.JacobiMatrix_leg(Q_legL_, UnitVec_legL_, P_legL_waist_standard_);

        // 各関節速度の計算
        if((t >= T_dsup/2 && t < T_dsup/2+0.05) || (t > (T_sup - T_dsup/2-0.05) && t <= (T_sup - T_dsup/2))) {
          jointVel_legR = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
          jointVel_legL = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        }
        else {
          jointVel_legR = Jacobi_legR_.inverse()*CoG_3D_Vel;
          jointVel_legL = Jacobi_legL_.inverse()*CoG_3D_Vel;
        }

        // 歩行パラメータの代入
        leg_joint_states_pat_ptr->joint_ang_pat_legR.push_back(Q_legR_);
        leg_joint_states_pat_ptr->joint_vel_pat_legR.push_back({jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]});  // eigen::vectorをstd::arrayに変換するためにこうしている。
        leg_joint_states_pat_ptr->joint_ang_pat_legL.push_back(Q_legL_);
        leg_joint_states_pat_ptr->joint_vel_pat_legL.push_back({jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]});
      }
      // 左脚支持期
      else if(walking_stabilization_ptr->zmp_pos_fix[walking_step][1] > 0) {
        // IK
        Q_legR_ = IK_.getIK(
          P_legR_waist_standard_,
          Foot_3D_Pos_Swing, 
          R_target_leg
        );
        Q_legL_ = IK_.getIK(
          P_legL_waist_standard_,
          Foot_3D_Pos,
          R_target_leg
        );

        // LOG:
        //WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL_, P_legL_waist_standard_, 6).transpose() << std::endl;

        // Jacobianの計算、Jacobianを記憶するクラス変数の更新
        // JacobiMatrix_leg(Q_legR_, Q_legL_);
        // Jacobi_legR_ = JacobiMatrix_leg(Q_legR_, UnitVec_legR_, P_legR_waist_standard_);
        // Jacobi_legL_ = JacobiMatrix_leg(Q_legL_, UnitVec_legL_, P_legL_waist_standard_);
        Jacobi_legR_ = Jacobian_.JacobiMatrix_leg(Q_legR_, UnitVec_legR_, P_legR_waist_standard_);
        Jacobi_legL_ = Jacobian_.JacobiMatrix_leg(Q_legL_, UnitVec_legL_, P_legL_waist_standard_);

        // 各関節速度の計算
        if((t >= T_dsup/2 && t < T_dsup/2+0.05) || (t > (T_sup - T_dsup/2-0.05) && t <= (T_sup - T_dsup/2))) {
          jointVel_legR = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
          jointVel_legL = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        }
        else {
          jointVel_legR = Jacobi_legR_.inverse()*CoG_3D_Vel_Swing;
          jointVel_legL = Jacobi_legL_.inverse()*CoG_3D_Vel;
        }

        // 歩行パラメータの代入
        leg_joint_states_pat_ptr->joint_ang_pat_legR.push_back(Q_legR_);  // 遊脚
        leg_joint_states_pat_ptr->joint_vel_pat_legR.push_back({jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]});
        leg_joint_states_pat_ptr->joint_ang_pat_legL.push_back(Q_legL_);  // 支持脚
        leg_joint_states_pat_ptr->joint_vel_pat_legL.push_back({jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]});
      }
      // 右脚支持期
      else if(walking_stabilization_ptr->zmp_pos_fix[walking_step][1] < 0) {
        // IK
        Q_legR_ = IK_.getIK(
          P_legR_waist_standard_,
          Foot_3D_Pos, 
          R_target_leg
        );
        Q_legL_ = IK_.getIK(
          P_legL_waist_standard_,
          Foot_3D_Pos_Swing,
          R_target_leg
        );

        // LOG:
        //WPG_log_FootTrajectory_FK << FK_.getFK(Q_legR_, P_legR_waist_standard_, 6).transpose() << " " << FK_.getFK(Q_legL_, P_legL_waist_standard_, 6).transpose() << std::endl;

        // Jacobianの計算、Jacobianを記憶するクラス変数の更新
        // JacobiMatrix_leg(Q_legR_, Q_legL_);
        // Jacobi_legR_ = JacobiMatrix_leg(Q_legR_, UnitVec_legR_, P_legR_waist_standard_);
        // Jacobi_legL_ = JacobiMatrix_leg(Q_legL_, UnitVec_legL_, P_legL_waist_standard_);
        Jacobi_legR_ = Jacobian_.JacobiMatrix_leg(Q_legR_, UnitVec_legR_, P_legR_waist_standard_);
        Jacobi_legL_ = Jacobian_.JacobiMatrix_leg(Q_legL_, UnitVec_legL_, P_legL_waist_standard_);

        // 各関節速度の計算
        if((t >= T_dsup/2 && t < T_dsup/2+0.05) || (t > (T_sup - T_dsup/2-0.05) && t <= (T_sup - T_dsup/2))) {
          jointVel_legR = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
          jointVel_legL = {12.26, 12.26, 12.26, 12.26, 12.26, 12.26};
        }
        else {
          jointVel_legR = Jacobi_legR_.inverse()*CoG_3D_Vel;
          jointVel_legL = Jacobi_legL_.inverse()*CoG_3D_Vel_Swing;
        }

        // 歩行パラメータの代入
        leg_joint_states_pat_ptr->joint_ang_pat_legR.push_back(Q_legR_);  // 支持脚
        leg_joint_states_pat_ptr->joint_vel_pat_legR.push_back({jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]});
        leg_joint_states_pat_ptr->joint_ang_pat_legL.push_back(Q_legL_);  // 遊脚
        leg_joint_states_pat_ptr->joint_vel_pat_legL.push_back({jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]});
      }

      // 更新
      control_step++;
      t += control_cycle;
      walking_time += control_cycle;

    }
    
    // LOG: Log file close
    //WPG_log_WalkingPttern.close();
    //WPG_log_FootTrajectory.close();
    //WPG_log_FootTrajectory_FK.close();
    // WPG_log_SwingTrajectory.close();

//==COPY==

    return leg_joint_states_pat_ptr;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(convert_to_joint_states::Default_ConvertToJointStates, control_plugin_base::ConvertToJointStates)