#include "walking_pattern_generator/LinearInvertedPendulumModel.hpp"

#include <fstream>

namespace walking_pattern_generator
{
  std::unique_ptr<control_plugin_base::WalkingPattern> WPG_LinearInvertedPendulumModel::walking_pattern_generator(
    const std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr
  ) {
    std::unique_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = std::make_unique<control_plugin_base::WalkingPattern>();

    // // LOG: Logを吐くファイルを指定
    // std::ofstream WPG_log_WalkingPttern;
    // std::string WPG_log_WalkingPttern_path = "src/Log/WPG_log_WalkingPattern.dat";
    // WPG_log_WalkingPttern.open(WPG_log_WalkingPttern_path, std::ios::out);

    // 制御周期
    // const float CONTROL_CYCLE_ = 0.01;  // [s]

    // 歩行パラメータの最終着地時間[s]を抽出
    float walking_time_max = WALKING_CYCLE_ * (foot_step_ptr->foot_pos.size()-1);

    // 時間, 時定数
    float t = 0;  // 0 ~ 支持脚切り替え時間
    //float WALKING_CYCLE_ = WALKING_CYCLE_;  // 支持脚切り替えタイミング. 歩行素片終端時間
    // float T_dsup = 0.5;  // 両脚支持期間
    float T_c = std::sqrt(WAIST_POS_Z_ / 9.81);  // 時定数

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
    // 歩行素片のパラメータ
    double x_bar = 0;
    double y_bar = 0;
    double dx_bar = 0;
    double dy_bar = 0;

    // 着地位置修正の最適化での重み
    int opt_weight_pos = 10;
    int opt_weight_vel = 1;
    // 最適化のときのマテリアル
    double D = opt_weight_pos * std::pow((std::cosh(WALKING_CYCLE_ / T_c) - 1), 2) + opt_weight_vel * std::pow((std::sinh(WALKING_CYCLE_ / T_c) / T_c), 2);  

    // time
    int control_step = 0;
    int walking_step = 0;
    float walking_time = 0;
    double S, C;  // sinh, cosh の短縮
    
  // 初期着地位置の修正
    // sinh, cosh
    S = std::sinh(WALKING_CYCLE_ / T_c);
    C = std::cosh(WALKING_CYCLE_ / T_c);
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
    walking_pattern_ptr->wc_foot_land_pos_ref.push_back({
      -1 * ((opt_weight_pos * (C - 1)) / D) * (x_d - C * CoG_2D_Pos_0[walking_step][0] - T_c * S * dx_0) - ((opt_weight_vel * S) / (T_c * D)) * (dx_d - (S / T_c) * CoG_2D_Pos_0[walking_step][0] - C * dx_0),
      -1 * ((opt_weight_pos * (C - 1)) / D) * (y_d - C * CoG_2D_Pos_0[walking_step][1] - T_c * S * dy_0) - ((opt_weight_vel * S) / (T_c * D)) * (dy_d - (S / T_c) * CoG_2D_Pos_0[walking_step][1] - C * dy_0)
    });


//=====歩行パターンの生成
    while(walking_time <= walking_time_max) {

      // sinh(Tsup/Tc), cosh(Tsup/Tc). 0 <= Tsup <= Tsup_max(=0.8)
      S = std::sinh(t / T_c);
      C = std::cosh(t / T_c);
      
      // 重心位置の計算
      walking_pattern_ptr->cc_cog_pos_ref.push_back({
        (CoG_2D_Pos_0[walking_step][0] - walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][0]) * C + T_c * dx_0 * S + walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][0],  // position_x
        (CoG_2D_Pos_0[walking_step][1] - walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][1]) * C + T_c * dy_0 * S + walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][1]  // position_y
      });
      // 重心速度の計算
      walking_pattern_ptr->cc_cog_vel_ref.push_back({
        ((CoG_2D_Pos_0[walking_step][0] - walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][0]) / T_c) * S + dx_0 * C,
        ((CoG_2D_Pos_0[walking_step][1] - walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][1]) / T_c) * S + dy_0 * C
      });

      // 支持脚切り替えの判定
      // BUG: t == 0.8 になっても、ifが実行されて、0.81になってしまう。応急処置で、WALKING_CYCLE_ - 0.01
      if(t < WALKING_CYCLE_ - 0.01) {
        // 値の更新
        t += CONTROL_CYCLE_;
        // 着地位置の維持（重心要素の配列の要素数と一致させるため）
        // walking_pattern_ptr->wc_foot_land_pos_ref.push_back(walking_pattern_ptr->wc_foot_land_pos_ref.back());
      }
      else if(t >= WALKING_CYCLE_ - 0.01) {
        // stepの更新
        walking_step++;
        
        // sinh(Tsup/Tc), cosh(Tsup/Tc). 特に意味はない。結局if内では、TsupはTsup_maxと等しいので。
        S = std::sinh(WALKING_CYCLE_ / T_c);
        C = std::cosh(WALKING_CYCLE_ / T_c);

        // 次の歩行素片の初期状態を定義
        CoG_2D_Pos_0.push_back({
          walking_pattern_ptr->cc_cog_pos_ref[control_step][0],
          walking_pattern_ptr->cc_cog_pos_ref[control_step][1]
        });
        dx_0 = walking_pattern_ptr->cc_cog_vel_ref[control_step][0];
        dy_0 = walking_pattern_ptr->cc_cog_vel_ref[control_step][1];

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
        walking_pattern_ptr->wc_foot_land_pos_ref.push_back({
          -1 * ((opt_weight_pos * (C - 1)) / D) * (x_d - C * CoG_2D_Pos_0[walking_step][0] - T_c * S * dx_0) - ((opt_weight_vel * S) / (T_c * D)) * (dx_d - (S / T_c) * CoG_2D_Pos_0[walking_step][0] - C * dx_0),
          -1 * ((opt_weight_pos * (C - 1)) / D) * (y_d - C * CoG_2D_Pos_0[walking_step][1] - T_c * S * dy_0) - ((opt_weight_vel * S) / (T_c * D)) * (dy_d - (S / T_c) * CoG_2D_Pos_0[walking_step][1] - C * dy_0)
        });
        
        // 値の更新
        t = 0.01;
      }

      // 値の更新
      control_step++;
      walking_time += CONTROL_CYCLE_;
    }
    // const int CONTROL_STEP_MAX = control_step;  // 制御周期数の最大値を記憶

    // std::cout << CONTROL_STEP_MAX << " " << control_step << " " << walking_pattern_ptr->cc_cog_pos_ref.size() << " " << std::endl;

    // Y座標値の修正（右足->胴体）
      // TODO: バカバカしいので、はじめから胴体投影点で歩行パターンを生成するようにしたい。
    for(long unsigned int step = 0; step < walking_pattern_ptr->cc_cog_pos_ref.size(); step++) {
      walking_pattern_ptr->cc_cog_pos_ref[step][1] -= foot_step_ptr->foot_pos[0][1];
    }
    for(long unsigned int step = 0; step < walking_pattern_ptr->wc_foot_land_pos_ref.size(); step++) {
      walking_pattern_ptr->wc_foot_land_pos_ref[step][1] -= foot_step_ptr->foot_pos[0][1];
    }

    // // LOG: plot用
    //   // TODO: 胴体投影点で歩行パターンを生成すればココも不要になって、生成のループ内に記述できる。
    // walking_step = 0;
    // walking_time = 0;
    // control_step = 0;
    // t = 0;
    // while(walking_time <= walking_time_max) {
    //   if(t < WALKING_CYCLE_ - 0.01) {
    //     t += CONTROL_CYCLE_;
    //   }
    //   else if(t >= WALKING_CYCLE_ - 0.01) {
    //     walking_step++;
    //     t = 0.01;
    //   }
    //   WPG_log_WalkingPttern << walking_pattern_ptr->cc_cog_pos_ref[control_step][0] << " " << walking_pattern_ptr->cc_cog_pos_ref[control_step][1] << " " 
    //             << walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][0] << " " << walking_pattern_ptr->wc_foot_land_pos_ref[walking_step][1] << " " 
    //             << foot_step_ptr->foot_pos[walking_step][0] << " " << foot_step_ptr->foot_pos[walking_step][1]-(foot_step_ptr->foot_pos[0][1])
    //   << std::endl;
    //   control_step++;
    //   walking_time += CONTROL_CYCLE_;
    // }

    // // LOG: Log file close
    // WPG_log_WalkingPttern.close();

    return walking_pattern_ptr;
  }

  WPG_LinearInvertedPendulumModel::WPG_LinearInvertedPendulumModel() {
    node_ptr_ = rclcpp::Node::make_shared("WalkingPatternGenerator");
    client_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "RobotParameterServer");
    // param: control
    CONTROL_CYCLE_ = client_param_->get_parameter<double>("control_times.control_cycle");
    WALKING_CYCLE_ = client_param_->get_parameter<double>("control_times.walking_cycle");
    WAIST_POS_Z_ = client_param_->get_parameter<double>("control_constant.waist_pos_z");

    RCLCPP_INFO(node_ptr_->get_logger(), "Start Up WalkingPatternGenerator.");
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(walking_pattern_generator::WPG_LinearInvertedPendulumModel, control_plugin_base::WalkingPatternGenerator)