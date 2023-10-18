#include "walking_pattern_generator/LinearInvertedPendulumModel.hpp"

#include <fstream>

namespace walking_pattern_generator
{
  std::unique_ptr<control_plugin_base::WalkingPattern> WPG_LinearInvertedPendulumModel::walking_pattern_generator(
    const std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr
  ) {
    std::unique_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = std::make_unique<control_plugin_base::WalkingPattern>();

    // LOG: Logを吐くファイルを指定
    std::ofstream WPG_log_WalkingPttern;
    std::string WPG_log_WalkingPttern_path = "src/Log/WPG_log_WalkingPattern.dat";
    WPG_log_WalkingPttern.open(WPG_log_WalkingPttern_path, std::ios::out);

    // 制御周期
    float control_cycle = 0.01;  // [s]

    // 歩行パラメータの最終着地時間[s]を抽出
    float walking_time_max = foot_step_ptr->walking_step_time * (foot_step_ptr->foot_pos.size()-1);

    // 重心位置・速度を保持する変数（重心は腰に位置するものとする）
    // std::vector<std::array<double, 2>> walking_pattern_ptr->cog_pos_ref;  // {{x0,y0},{x1,y1},{x2,y2}}
    // std::vector<std::array<double, 2>> CoG_2D_Pos_local;
    // std::vector<std::array<double, 2>> walking_pattern_ptr->cog_vel_ref;

    // 時間, 時定数
    float t = 0;  // 0 ~ 支持脚切り替え時間
    float T_sup = foot_step_ptr->walking_step_time;  // 支持脚切り替えタイミング. 歩行素片終端時間
    // float T_dsup = 0.5;  // 両脚支持期間
    float T_c = std::sqrt(foot_step_ptr->waist_height / 9.81);  // 時定数

    // 歩行素片の始端の重心位置・速度 (World座標系)
    std::vector<std::array<double, 2>> CoG_2D_Pos_0;
    CoG_2D_Pos_0.push_back({0, 0.037});
    double dx_0 = 0;
    double dy_0 = 0;
    // 理想の重心位置・速度 (World座標系)
    double x_d = 0;
    double y_d = 0;
    double dx_d = 0;
    double dy_d = 0;
    // 支持脚着地位置・修正着地位置
    // std::vector<std::array<double, 2>> walking_pattern_ptr->foot_pos_ref;
    // 歩行素片のパラメータ
    double x_bar = 0;
    double y_bar = 0;
    double dx_bar = 0;
    double dy_bar = 0;

    // 着地位置修正の最適化での重み
    int opt_weight_pos = 10;
    int opt_weight_vel = 1;
    // 最適化のときのマテリアル
    double D = opt_weight_pos * std::pow((std::cosh(T_sup / T_c) - 1), 2) + opt_weight_vel * std::pow((std::sinh(T_sup / T_c) / T_c), 2);  

    // loop. 0: control_cycle: walking_time_max
    int control_step = 0;
    int walking_step = 0;
    float walking_time = 0;
    double S, C;  // sinh, cosh の短縮
    
    // 初期着地位置はfoot_step_ptr->foot_posと同等なので、そちらを参照。

  // 初期着地位置の修正
    // sinh, cosh
    S = std::sinh(T_sup / T_c);
    C = std::cosh(T_sup / T_c);
    // 次の歩行素片のパラメータを計算 
    x_bar = (foot_step_ptr->foot_pos[walking_step + 1][0] - foot_step_ptr->foot_pos[walking_step][0]) / 2;
    y_bar = (foot_step_ptr->foot_pos[walking_step + 1][1] - foot_step_ptr->foot_pos[walking_step][1]) / 2;
    dx_bar = ((C + 1) / (T_c * S)) * x_bar;
    dy_bar = ((C - 1) / (T_c * S)) * y_bar;
    // 次の歩行素片の最終状態の目標値
    x_d = foot_step_ptr->foot_pos[walking_step][0] + x_bar;
    y_d = foot_step_ptr->foot_pos[walking_step][1] + y_bar;
    dx_d = dx_bar;
    dy_d = dy_bar;
    // 評価関数を最小化する着地位置の計算
    walking_pattern_ptr->foot_pos_ref.push_back({
      -1 * ((opt_weight_pos * (C - 1)) / D) * (x_d - C * CoG_2D_Pos_0[walking_step][0] - T_c * S * dx_0) - ((opt_weight_vel * S) / (T_c * D)) * (dx_d - (S / T_c) * CoG_2D_Pos_0[walking_step][0] - C * dx_0),
      -1 * ((opt_weight_pos * (C - 1)) / D) * (y_d - C * CoG_2D_Pos_0[walking_step][1] - T_c * S * dy_0) - ((opt_weight_vel * S) / (T_c * D)) * (dy_d - (S / T_c) * CoG_2D_Pos_0[walking_step][1] - C * dy_0)
    });


//=====歩行パターンの生成
    while(walking_time <= walking_time_max) {

      // sinh(Tsup/Tc), cosh(Tsup/Tc). 0 <= Tsup <= Tsup_max(=0.8)
      S = std::sinh(t / T_c);
      C = std::cosh(t / T_c);
      
      // 重心位置の計算
      walking_pattern_ptr->cog_pos_ref.push_back({
        (CoG_2D_Pos_0[walking_step][0] - walking_pattern_ptr->foot_pos_ref[walking_step][0]) * C + T_c * dx_0 * S + walking_pattern_ptr->foot_pos_ref[walking_step][0],  // position_x
        (CoG_2D_Pos_0[walking_step][1] - walking_pattern_ptr->foot_pos_ref[walking_step][1]) * C + T_c * dy_0 * S + walking_pattern_ptr->foot_pos_ref[walking_step][1]  // position_y
      });
      // 重心速度の計算
      walking_pattern_ptr->cog_vel_ref.push_back({
        ((CoG_2D_Pos_0[walking_step][0] - walking_pattern_ptr->foot_pos_ref[walking_step][0]) / T_c) * S + dx_0 * C,
        ((CoG_2D_Pos_0[walking_step][1] - walking_pattern_ptr->foot_pos_ref[walking_step][1]) / T_c) * S + dy_0 * C
      });

      // 支持脚切り替えの判定
      // BUG: t == 0.8 になっても、ifが実行されて、0.81になってしまう。応急処置で、T_sup - 0.01
      if(t < T_sup - 0.01) {
        // 値の更新
        t += control_cycle;
        // 着地位置の維持（重心要素の配列の要素数と一致させるため）
        // walking_pattern_ptr->foot_pos_ref.push_back(walking_pattern_ptr->foot_pos_ref.back());
      }
      else if(t >= T_sup - 0.01) {
        // stepの更新
        walking_step++;
        
        // sinh(Tsup/Tc), cosh(Tsup/Tc). 特に意味はない。結局if内では、TsupはTsup_maxと等しいので。
        S = std::sinh(T_sup / T_c);
        C = std::cosh(T_sup / T_c);

        // 次の歩行素片の初期状態を定義
        CoG_2D_Pos_0.push_back({
          walking_pattern_ptr->cog_pos_ref[control_step][0],
          walking_pattern_ptr->cog_pos_ref[control_step][1]
        });
        dx_0 = walking_pattern_ptr->cog_vel_ref[control_step][0];
        dy_0 = walking_pattern_ptr->cog_vel_ref[control_step][1];

        // 次の歩行素片のパラメータを計算 
        x_bar = (foot_step_ptr->foot_pos[walking_step + 1][0] - foot_step_ptr->foot_pos[walking_step][0]) / 2;
        y_bar = (foot_step_ptr->foot_pos[walking_step + 1][1] - foot_step_ptr->foot_pos[walking_step][1]) / 2;
        dx_bar = ((C + 1) / (T_c * S)) * x_bar;
        dy_bar = ((C - 1) / (T_c * S)) * y_bar;

        // 次の歩行素片の最終状態の目標値
        x_d = foot_step_ptr->foot_pos[walking_step][0] + x_bar;
        y_d = foot_step_ptr->foot_pos[walking_step][1] + y_bar;
        dx_d = dx_bar;
        dy_d = dy_bar;

        // 評価関数を最小化する着地位置の計算
        walking_pattern_ptr->foot_pos_ref.push_back({
          -1 * ((opt_weight_pos * (C - 1)) / D) * (x_d - C * CoG_2D_Pos_0[walking_step][0] - T_c * S * dx_0) - ((opt_weight_vel * S) / (T_c * D)) * (dx_d - (S / T_c) * CoG_2D_Pos_0[walking_step][0] - C * dx_0),
          -1 * ((opt_weight_pos * (C - 1)) / D) * (y_d - C * CoG_2D_Pos_0[walking_step][1] - T_c * S * dy_0) - ((opt_weight_vel * S) / (T_c * D)) * (dy_d - (S / T_c) * CoG_2D_Pos_0[walking_step][1] - C * dy_0)
        });
        
        // 値の更新
        t = 0.01;
      }

      // 値の更新
      control_step++;
      walking_time += control_cycle;
    }

    // Y座標値の修正（右足->胴体）
      // TODO: バカバカしいので、はじめから胴体投影点で歩行パターンを生成するようにしたい。
    for(long unsigned int step = 0; step < walking_pattern_ptr->cog_pos_ref.size(); step++) {
      walking_pattern_ptr->cog_pos_ref[step][1] -= foot_step_ptr->foot_pos[0][1];
    }
    for(long unsigned int step = 0; step < walking_pattern_ptr->foot_pos_ref.size(); step++) {
      walking_pattern_ptr->foot_pos_ref[step][1] -= foot_step_ptr->foot_pos[0][1];
    }

    // LOG: plot用
      // TODO: 胴体投影点で歩行パターンを生成すればココも不要になって、生成のループ内に記述できる。
    walking_step = 0;
    walking_time = 0;
    control_step = 0;
    t = 0;
    while(walking_time <= walking_time_max) {
      if(t < T_sup - 0.01) {
        t += control_cycle;
      }
      else if(t >= T_sup - 0.01) {
        walking_step++;
        t = 0.01;
      }
      WPG_log_WalkingPttern << walking_pattern_ptr->cog_pos_ref[control_step][0] << " " << walking_pattern_ptr->cog_pos_ref[control_step][1] << " " 
                << walking_pattern_ptr->foot_pos_ref[walking_step][0] << " " << walking_pattern_ptr->foot_pos_ref[walking_step][1] << " " 
                << foot_step_ptr->foot_pos[walking_step][0] << " " << foot_step_ptr->foot_pos[walking_step][1]-(foot_step_ptr->foot_pos[0][1])
      << std::endl;
      control_step++;
      walking_time += control_cycle;
    }

    // LOG: Log file close
    WPG_log_WalkingPttern.close();

    // std::cout << "Here is wpg_linear_inverted_pendulum_model plugin." << std::endl;

// TODO: CTJSから足の軌道計算プログラムを移行
//     while(walking_time <= walking_time_max) {

//       // 支持脚切替タイミングの判定
//       if(t >= T_sup - 0.01) {
//         // 支持脚切替のための更新
//         t = 0;
//         walking_step++;
//       }

//       // // 位置の基準を修正（Y軸基準を右足裏から胴体中心へ）
//       // walking_stabilization_ptr->cog_pos_fix[control_step][1] -= Initwalking_stabilization_ptr->zmp_pos_fixy;

//       // 遊脚軌道（正弦波）の計算
//       // TODO: 両脚支持期間は支持脚切替時に重心速度が急激に変化しないようにするために設けるものである。
//       if(t >= T_dsup/2 && t <= T_sup-T_dsup/2) {  // 片足支持期
//         old_swing_trajectory = swing_trajectory;
//         swing_trajectory = height_leg_lift * std::sin((3.141592/(T_sup-T_dsup))*(t-T_dsup/2));  
//         vel_swing_trajectory = ((swing_trajectory - old_swing_trajectory) / control_cycle);
//       }
//       else {  // 両脚支持期
//         swing_trajectory = 0.0;
//         old_swing_trajectory = 0.0;
//         vel_swing_trajectory = 0.0;
//       }

//       // LOG: 遊脚軌道に関するlogの取得
//       // WPG_log_SwingTrajectory << swing_trajectory << " " << old_swing_trajectory << " " << (swing_trajectory-old_swing_trajectory) << std::endl;      

// //=====足の軌道計算
// // TODO: ココはWalkingPatternGeneratorに実装するべき。そのほうがコードもまとまる。
//   // walking_stepとcontrol_stepを使っている、かつstepの前後も用いているので、毎step呼ばれて計算を行うのが汚くなる。
//       if(walking_stabilization_ptr->zmp_pos_fix[walking_step][1] == 0) {  // 歩行開始時、終了時
//         int ref_ws; 
//         if(walking_step == 0) {  // 歩行開始時
//           ref_ws = walking_step+1;
//         }
//         else {  // 開始時以外
//           ref_ws = walking_step-1;
//         }
//         if(walking_stabilization_ptr->zmp_pos_fix[ref_ws][1] >= 0) {  // 左脚支持
//           Foot_3D_Pos = {  // 左足
//             walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
//             0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg
//           };
//           Foot_3D_Pos_Swing = {  // 右足
//             walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
//             -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg
//           };
//         }
//         else if(walking_stabilization_ptr->zmp_pos_fix[ref_ws][1] < 0) {  // 右脚支持
//           Foot_3D_Pos = {  // 右足
//             walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
//             -0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg
//           };
//           Foot_3D_Pos_Swing = {  // 左足
//             walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
//             0.037-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg
//           };
//         }
//       }
//       else if(walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1] == 0) {  // 歩行開始から1step後
//         // 支持脚
//         Foot_3D_Pos = {  
//           walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
//           walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y 
//           -length_leg  // z 
//         };
//         // 遊脚
//         if(t <= T_dsup/2) {  // 両脚支持（前半）
//           Foot_3D_Pos_Swing = {
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],
//             walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
//             -length_leg
//           };
//         }
//         else if(t >= T_sup-T_dsup/2) {  // 両脚支持（後半）
//           Foot_3D_Pos_Swing = {
//             walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
//             walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
//             -length_leg
//           };
//         }
//         else {  // 片脚支持
//           Foot_3D_Pos_Swing = {
//             ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup/2)/(T_sup-T_dsup))),
//             walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg + swing_trajectory // z (遊脚軌道をzから引く) 
//           };
//         }

//       }
//       else if(walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1] == 0) {  // 歩行終了から1step前
//         // 支持脚
//         Foot_3D_Pos = {
//           walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
//           walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y  
//           -length_leg  // z 
//         };
//         // 遊脚
//         if(t <= T_dsup/2) {  // 両脚支持（前半）
//           Foot_3D_Pos_Swing = {
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],  
//             -length_leg
//           };
//         }
//         else if(t >= T_sup-T_dsup/2) {  // 両脚支持（後半）
//           Foot_3D_Pos_Swing = {
//             walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
//             -length_leg
//           };
//         }
//         else {  // 片脚支持
//           Foot_3D_Pos_Swing = {
//             ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup/2)/(T_sup-T_dsup)))-(walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]), 
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg + swing_trajectory
//           };
//         }
//       }
//       else {  // 歩行中
//         // 支持脚
//         Foot_3D_Pos = {
//           walking_stabilization_ptr->zmp_pos_fix[walking_step][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0],  // x 
//           walking_stabilization_ptr->zmp_pos_fix[walking_step][1]-walking_stabilization_ptr->cog_pos_fix[control_step][1],  // y
//           -length_leg  // z 
//         };
//         // 遊脚
//         if(t <= T_dsup/2) {  // 両脚支持（前半）
//           Foot_3D_Pos_Swing = {
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
//             -length_leg
//           };
//         }
//         else if(t >= T_sup-T_dsup/2) {  // 両脚支持（後半）
//           Foot_3D_Pos_Swing = {
//             walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->cog_pos_fix[control_step][0], 
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1], 
//             -length_leg
//           };
//         }
//         else {  // 片脚支持
//           Foot_3D_Pos_Swing = {
//             ((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0])*((t-T_dsup/2)/(T_sup-T_dsup)))-((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][0]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][0]) / 2),  
//             walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1]+((walking_stabilization_ptr->zmp_pos_fix[walking_step+1][1]-walking_stabilization_ptr->zmp_pos_fix[walking_step-1][1])*(t/(T_sup)))-walking_stabilization_ptr->cog_pos_fix[control_step][1],
//             -length_leg + swing_trajectory 
//           };
//         }
//       }



    return walking_pattern_ptr;
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(walking_pattern_generator::WPG_LinearInvertedPendulumModel, control_plugin_base::WalkingPatternGenerator)