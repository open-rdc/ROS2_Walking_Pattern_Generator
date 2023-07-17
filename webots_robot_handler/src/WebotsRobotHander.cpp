#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include <fstream>  // Logをファイルに吐くため
#include <rmw/qos_profiles.h>
#include "msgs_package/msg/control_output.hpp"
#include "msgs_package/msg/feedback.hpp"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>

#include "Eigen/Dense"
#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"

using namespace std::chrono_literals;
using namespace Eigen;

namespace webots_robot_handler
{
  static const rmw_qos_profile_t custom_qos_profile =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
    1,  // History(keep_last) Depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
    RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
    RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
    RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
    false  // avoid_ros_namespace_conventions
  };

  void WebotsRobotHandler::DEBUG_ParameterSetting() {
    // protoに沿った各モータの名前
    motors_name_ = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"),   // arm
                    ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"),   //leg
                    ("Neck"), ("Head")};  //body
    // 初期姿勢を設定. init joints ang [rad]. corresponding to motors_name
    initJointAng_ = {0, 0, -0.5, 0.5, -1, 1,   // arm
                    0, 0, 0, 0, -3.14/8, 3.14/8, 3.14/4, -3.14/4, 3.14/8, -3.14/8, 0, 0,   // leg
                    0, 0.26};  // body
    // 初期姿勢に移行する時の角速度.  init joints vel [rad/s]
    initJointVel_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5,  // arm
                    0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5,  // log
                    0.5, 0.5};  // body

    // 脚のラベル一覧
    jointNum_legR_ = {6, 8, 10, 12, 14, 16};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(right leg)
    jointNum_legL_ = {7, 9, 11, 13, 15, 17};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(left leg)
    // IKなどは右手系で解いているが、ロボットで左手系を採用している関節が幾つかある。それに対応する為の配列。
    jointAng_posi_or_nega_legR_ = {-1, -1, 1, 1, -1, 1};  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
    jointAng_posi_or_nega_legL_ = {-1, -1, -1, -1, 1, 1}; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)

    // TODO: 単位ベクトルの設定。これもParameterやURDF, Protoから得たい。
    UnitVec_legR_ = {  // legR joint unit vector
      Vector3d(0, 0, 1),
      Vector3d(1, 0, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(1, 0, 0)
    };
    UnitVec_legL_ = {  // legL joint unit vector
      Vector3d(0, 0, 1),
      Vector3d(1, 0, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(1, 0, 0)      
    };

    // 脚の関節位置の読み込み。基準(0, 0, 0)は喉仏あたり
    P_legR_ = {  // 右脚
        Vector3d(-0.005, -0.037, -0.1222),  // o(基準) -> 1
        Vector3d(0, 0, 0),  // 1 -> 2
        Vector3d(0, 0, 0),  // 2 -> 3
        Vector3d(0, 0, -0.093),  // 3 -> 4
        Vector3d(0, 0, -0.093),  // 4 -> 5
        Vector3d(0, 0, 0),  // 5 -> 6
        Vector3d(0, 0, 0)  // 6 -> a(足裏)
    };
    P_legL_ = {  // 左脚
        Vector3d(-0.005, 0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    // 脚の関節位置。基準を腰（ｚ軸が股関節と等しい）に修正
    P_legR_waist_standard_ = {  // 右脚
        Vector3d(-0.005, -0.037, 0),  // o(基準) -> 1
        Vector3d(0, 0, 0),  // 1 -> 2
        Vector3d(0, 0, 0),  // 2 -> 3
        Vector3d(0, 0, -0.093),  // 3 -> 4
        Vector3d(0, 0, -0.093),  // 4 -> 5
        Vector3d(0, 0, 0),  // 5 -> 6
        Vector3d(0, 0, 0)  // 6 -> a(足裏)
    };
    P_legL_waist_standard_ = {  // 左脚
        Vector3d(-0.005, 0.037, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    R_target_leg << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;  // Legの末端の回転行列。床面と並行にしたいので、ただの単位行列。

    // Dynamic Gait ====
    weight_ = 3.0;  // [kg]
    length_leg_ = 171.856 / 1000;  // [m] ちょっと中腰。特異点を回避。直立：219.5[mm]
    // TODO: 足踏み。歩行する着地位置を計算して適用すべき
    LandingPosition_ = {{0.0, 0.0, 0.037},  // 歩行パラメータからの着地位置(time, x, y)
                        {0.8, 0.0, 0.0},  // 元は、0.037. 基準点を変えている. 
                        {1.6, 0.0, 0.074},  // TODO: IKを解くときなど、WPを計算するとき以外は基準がずれるので、修正するように。
                        {2.4, 0.0, 0.0},  // TODO: そもそもコレの基準点を胴体の真下でも通じるようにするべき。
                        {3.2, 0.0, 0.074},
                        {4.0, 0.0, 0.0},
                        {4.8, 0.0, 0.074},
                        {5.6, 0.0, 0.037},
                        {6.4, 0.0, 0.037}};
    // LandingPosition_ = {{0.0, 0.0, 0.037},  // 歩行パラメータからの着地位置(time, x, y)
    //                     {0.8, 0.025, 0.074},  // 元は、0.037. 基準点を変えている. 
    //                     {1.6, 0.05, 0.0},  // TODO: IKを解くときなど、WPを計算するとき以外は基準がずれるので、修正するように。
    //                     {2.4, 0.075, 0.074},  // TODO: そもそもコレの基準点を胴体の真下でも通じるようにするべき。
    //                     {3.2, 0.1, 0.0},
    //                     {4.0, 0.125, 0.074},
    //                     {4.8, 0.15, 0.037},
    //                     {5.6, 0.15, 0.037}};

    // DEBUG: Jacobian関数のテスト
    // Q_legR_ = {0, 0, -3.14/8, 3.14/4, -3.14/8, 0};
    // Q_legL_ = {0, 0, -3.14/8, 3.14/4, -3.14/8, 0};
    // JacobiMatrix_leg(Q_legR_, Q_legL_);
    // std::cout << "\n" << Jacobi_legR_ << "\n" << std::endl;
    // std::cout << "\n" << Jacobi_legL_ << "\n" << std::endl;
  }

  // TODO: kinematics node でも作って、共有ライブラリにFK・IKともに入れたほうが良いと思う。
  // TODO: 返り値をVoidではなく、Jacobianを返すべき。分かりづらい。
  void WebotsRobotHandler::JacobiMatrix_leg(std::array<double, 6> Q_legR, std::array<double, 6> Q_legL) {
    Jacobi_legR_ = MatrixXd::Zero(6, UnitVec_legR_.max_size());
    Jacobi_legL_ = MatrixXd::Zero(6, UnitVec_legR_.max_size());

    for(int joint_point = 0; joint_point < int(UnitVec_legR_.max_size()); joint_point++) {
      P_FK_legR_[joint_point] = FK_.getFK(Q_legR, P_legR_waist_standard_, joint_point);
      P_FK_legL_[joint_point] = FK_.getFK(Q_legL, P_legL_waist_standard_, joint_point);
      // std::cout << P_FK_legR_[joint_point] << std::endl;
      // std::cout << P_FK_legL_[joint_point] << std::endl;
    }

    Vector3d mat_legR = Vector3d::Zero(3);
    Vector3d mat_legL = Vector3d::Zero(3);
    Vector3d pt_P_legR = Vector3d::Zero(3);
    Vector3d pt_P_legL = Vector3d::Zero(3);
    for(int tag = 0; tag < int(UnitVec_legR_.max_size()); tag++) {
      if(tag == int(UnitVec_legR_.max_size()-1)) {
        mat_legR = Vector3d::Zero(3);
        mat_legL = Vector3d::Zero(3);
      }
      else { 
        // P_FK_legR_[int(UnitVec_legR_.max_size())-1]: 股関節の座標を取得（基準座標から股関節までの距離を含まないFKの結果を取得）
        pt_P_legR = P_FK_legR_[int(UnitVec_legR_.max_size())-1] - P_FK_legR_[tag];
        pt_P_legL = P_FK_legL_[int(UnitVec_legL_.max_size())-1] - P_FK_legL_[tag];
        // std::cout << "pt_P_legR: " << pt_P_legR.transpose() << ", " << P_FK_legR_[int(UnitVec_legR_.max_size())-1].transpose() << ", " << P_FK_legR_[tag].transpose() << std::endl;
        // std::cout << "pt_P_legL: " << pt_P_legL.transpose() << ", " << P_FK_legL_[int(UnitVec_legL_.max_size())-1].transpose() << ", " << P_FK_legL_[tag].transpose() << std::endl;
        mat_legR = UnitVec_legR_[tag].cross(pt_P_legR);
        mat_legL = UnitVec_legL_[tag].cross(pt_P_legL);
      }

      for(int i = 0; i < 3; i++) {
        Jacobi_legR_(i, tag) = mat_legR[i];
        Jacobi_legR_(i+3, tag) = UnitVec_legR_[tag][i];
        Jacobi_legL_(i, tag) = mat_legL[i];
        Jacobi_legL_(i+3, tag) = UnitVec_legL_[tag][i];
      }
    }
  }

  // TODO: 歩行パターンを生成する
  void WebotsRobotHandler::WalkingPatternGenerate() {

    // DEBUG: Logを吐くファイルを指定
    std::ofstream WPG_log_WalkingPttern;
    std::string WPG_log_WalkingPttern_name = "src/Log/WPG_log_WalkingPattern.dat";
    WPG_log_WalkingPttern.open(WPG_log_WalkingPttern_name, std::ios::out);
    std::ofstream WPG_log_FootTrajectory;
    std::string WPG_log_FootTrajectory_name = "src/Log/WPG_log_FootTrajectory.dat";
    WPG_log_FootTrajectory.open(WPG_log_FootTrajectory_name, std::ios::out);

    // 制御周期
    float control_cycle = 0.01;  // [s]

    // 歩行パラメータの最終着地時間[s]を抽出
    float walking_time_max = LandingPosition_[LandingPosition_.size()-1][0];  // TODO: 無駄な変数なので消すべき。わかりやすさ重視 

    // 重心位置・速度を保持する変数（重心は腰に位置するものとする）
    std::vector<std::array<double, 2>> CoG_2D_Pos_world;  // {{x0,y0},{x1,y1},{x2,y2}}
    std::vector<std::array<double, 2>> CoG_2D_Pos_local;
    std::vector<std::array<double, 2>> CoG_2D_Vel;

    // 時間, 時定数
    float t = 0;  // 0 ~ 支持脚切り替え時間
    float T_sup = LandingPosition_[1][0];  // 0.8. 支持脚切り替えタイミング. 歩行素片終端時間
    float T_dsup = 0.2;
    float T_c = std::sqrt(length_leg_ / 9.81);  // 時定数

    // 歩行素片の始端の重心位置・速度 (World座標系)
    double x_0 = 0;
    double y_0 = 0.037;
    double dx_0 = 0;
    double dy_0 = 0;
    // 理想の重心位置・速度 (World座標系)
    double x_d = 0;
    double y_d = 0;
    double dx_d = 0;
    double dy_d = 0;
    // 支持脚着地位置・修正着地位置
    double p_x_fix = 0;
    double p_y_fix = 0;
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

    // 初期着地位置の取得
    p_x_fix = LandingPosition_[walking_step][1];
    p_y_fix = LandingPosition_[walking_step][2];
    
  // 初期着地位置の修正
    // sinh, cosh
    S = std::sinh(T_sup / T_c);
    C = std::cosh(T_sup / T_c);
    // 次の歩行素片のパラメータを計算 
    x_bar = (LandingPosition_[walking_step + 1][1] - LandingPosition_[walking_step][1]) / 2;
    y_bar = (LandingPosition_[walking_step + 1][2] - LandingPosition_[walking_step][2]) / 2;
    dx_bar = ((C + 1) / (T_c * S)) * x_bar;
    dy_bar = ((C - 1) / (T_c * S)) * y_bar;
    // 次の歩行素片の最終状態の目標値
    x_d = p_x_fix + x_bar;
    y_d = p_y_fix + y_bar;
    dx_d = dx_bar;
    dy_d = dy_bar;
    // 評価関数を最小化する着地位置の計算
    p_x_fix = -1 * ((opt_weight_pos * (C - 1)) / D) * (x_d - C * x_0 - T_c * S * dx_0) - ((opt_weight_vel * S) / (T_c * D)) * (dx_d - (S / T_c) * x_0 - C * dx_0);
    p_y_fix = -1 * ((opt_weight_pos * (C - 1)) / D) * (y_d - C * y_0 - T_c * S * dy_0) - ((opt_weight_vel * S) / (T_c * D)) * (dy_d - (S / T_c) * y_0 - C * dy_0);

    // 歩行パターンの生成
    while(walking_time <= walking_time_max) {
      // 行を追加
      CoG_2D_Pos_world.push_back({0, 0});
      CoG_2D_Pos_local.push_back({0, 0});
      CoG_2D_Vel.push_back({0, 0});

      // sinh(Tsup/Tc), cosh(Tsup/Tc). 0 <= Tsup <= Tsup_max(=0.8)
      S = std::sinh(t / T_c);
      C = std::cosh(t / T_c);
      
      // 重心位置の計算
      CoG_2D_Pos_world[control_step][0] = (x_0 - p_x_fix) * C + T_c * dx_0 * S + p_x_fix;  // position_x
      CoG_2D_Pos_world[control_step][1] = (y_0 - p_y_fix) * C + T_c * dy_0 * S + p_y_fix;  // position_y
      CoG_2D_Pos_local[control_step][0] = (x_0 - p_x_fix) * C + T_c * dx_0 * S;  // position_x
      CoG_2D_Pos_local[control_step][1] = (y_0 - p_y_fix) * C + T_c * dy_0 * S + p_y_fix;  // position_y
      // 重心速度の計算
      CoG_2D_Vel[control_step][0] = ((x_0 - p_x_fix) / T_c) * S + dx_0 * C;
      CoG_2D_Vel[control_step][1] = ((y_0 - p_y_fix) / T_c) * S + dy_0 * C;

      // 支持脚切り替えの判定
      // BUG: t == 0.8 になっても、ifが実行されて、0.81になってしまう。応急処置で、T_sup - 0.01
      if(t < T_sup - 0.01) {
        // 値の更新
        t += control_cycle;
      }
      else if(t >= T_sup - 0.01) {
        // stepの更新
        walking_step++;
        
        // sinh(Tsup/Tc), cosh(Tsup/Tc). 特に意味はない。結局if内では、TsupはTsup_maxと等しいので。
        S = std::sinh(T_sup / T_c);
        C = std::cosh(T_sup / T_c);

        // 次の着地位置を取得
        p_x_fix = LandingPosition_[walking_step][1];
        p_y_fix = LandingPosition_[walking_step][2];

        // 次の歩行素片の初期状態を定義
        x_0 = CoG_2D_Pos_world[control_step][0];
        y_0 = CoG_2D_Pos_world[control_step][1];
        dx_0 = CoG_2D_Vel[control_step][0];
        dy_0 = CoG_2D_Vel[control_step][1];

        // 次の歩行素片のパラメータを計算 
        x_bar = (LandingPosition_[walking_step + 1][1] - LandingPosition_[walking_step][1]) / 2;
        y_bar = (LandingPosition_[walking_step + 1][2] - LandingPosition_[walking_step][2]) / 2;
        dx_bar = ((C + 1) / (T_c * S)) * x_bar;
        dy_bar = ((C - 1) / (T_c * S)) * y_bar;

        // 次の歩行素片の最終状態の目標値
        x_d = p_x_fix + x_bar;
        y_d = p_y_fix + y_bar;
        dx_d = dx_bar;
        dy_d = dy_bar;

        // 評価関数を最小化する着地位置の計算
        p_x_fix = -1 * ((opt_weight_pos * (C - 1)) / D) * (x_d - C * x_0 - T_c * S * dx_0) - ((opt_weight_vel * S) / (T_c * D)) * (dx_d - (S / T_c) * x_0 - C * dx_0);
        p_y_fix = -1 * ((opt_weight_pos * (C - 1)) / D) * (y_d - C * y_0 - T_c * S * dy_0) - ((opt_weight_vel * S) / (T_c * D)) * (dy_d - (S / T_c) * y_0 - C * dy_0);
        
        // 値の更新
        t = 0.01;
      }

      // DEBUG: plot用
      // TODO: 複数のファイルを読み込んで、複数種類のLogを吐くようにすべき。可変長の配列をMessageをPublishが扱えれば一番いいが。
      WPG_log_WalkingPttern << CoG_2D_Pos_world[control_step][0] << " " << CoG_2D_Pos_world[control_step][1]-(LandingPosition_[0][2]) << " " 
                << CoG_2D_Pos_local[control_step][0] << " " << CoG_2D_Pos_local[control_step][1]-(LandingPosition_[0][2]) << " " 
                << CoG_2D_Vel[control_step][0] << " " << CoG_2D_Vel[control_step][1] << " " 
                << p_x_fix << " " << p_y_fix-(LandingPosition_[0][2]) << " " 
                << LandingPosition_[walking_step][1] << " " << LandingPosition_[walking_step][2]-(LandingPosition_[0][2])
      << std::endl;

      // 値の更新
      control_step++;
      walking_time += control_cycle;
    }

    // DEBUG: Log file close
    WPG_log_WalkingPttern.close();

    // 遊脚軌道に必要な変数の定義
    float height_leg_lift = 0.07;  // 足上げ高さ [m]
    double swing_trajectory;  // 遊脚軌道の値を記録
    t = 0;
    walking_time = 0;
    walking_step = 0;
    control_step = 0;

    // 重心位置から遊脚軌道（正弦波）を引く。支持脚に応じて遊脚も切り替えるから、0.8[s]ごとに切り替える。
    // 遊脚軌道を反映するのは、IK解いて歩行パラメータを生成するloop内で一緒にやる。
    // 遊脚軌道の式：z = height_leg_lift * sin((pi / T_sup) * t)   0 <= t <= T_sup(=0.8[s])

    // IKと歩行パラメータの定義・遊脚軌道の反映
    Eigen::Vector<double, 3> Foot_3D_Pos;
    Eigen::Vector<double, 3> Foot_3D_Pos_Swing;
    Eigen::Vector<double, 6> CoG_3D_Vel;
    Eigen::Vector<double, 6> jointVel_legR;
    Eigen::Vector<double, 6> jointVel_legL;
    while(walking_time <= walking_time_max) {

      // 支持脚切替タイミングの判定
      if(t >= T_sup - 0.01) {
        // 支持脚切替のための更新
        t = 0;
        walking_step++;
      }

      // 遊脚軌道（正弦波）の計算
      // 両脚支持期間を無視するように条件分岐。両脚支持期間以外で正弦波を > 0 にしてやることで、片足支持にしている。
      // TODO: 両脚支持期間は支持脚切替時に重心速度が急激に変化しないようにするために設けるものである。
        // 歩行パターン生成時に、両脚支持期間を考慮すべきか？ただ単に両脚ともに地面についていれば良いのか？目標重心位置のYを0.037にすれば良いのか？
      if(t >= T_dsup/2 && t <= T_sup-T_dsup/2) {
        swing_trajectory = height_leg_lift * std::sin((3.141592/T_sup-T_dsup)*t-T_dsup/2);
      }
      else {
        swing_trajectory = 0;
      }
      

      // 重心位置を元とした足位置の定義
      // CHECKME: もっとキレイな書き方があるはず。修正すべき。今は、始まりの支持脚と終わりの支持脚が同じだからコレでOK。異なった場合も書くべき。
      // 歩行開始時、終了時
      if(LandingPosition_[walking_step][2] == 0.037) {
        int ref_ws;
        if(walking_step == 0) {  // 歩行開始時
          ref_ws = walking_step+1;
        }
        else {  // 開始時以外
          ref_ws = walking_step-1;
        }
        if(LandingPosition_[ref_ws][2]-LandingPosition_[ref_ws+1][1] >= 0) {  // 左脚支持
          Foot_3D_Pos = {  // 左足。
            LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],
            0.037-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),
            -length_leg_
          };
          Foot_3D_Pos_Swing = {  // 右足。歩行開始と終了では、直立状態の足の位置とから重心軌道を引いてやる。
            LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],
            -0.037-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),
            -length_leg_
          };
        }
        else if(LandingPosition_[ref_ws][2]-LandingPosition_[ref_ws+1][1] < 0) {  // 右脚支持
          Foot_3D_Pos = {  // 右足
            LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],
            -0.037-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),
            -length_leg_
          };
          Foot_3D_Pos_Swing = {  // 左足
            LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],
            0.037-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),
            -length_leg_
          };
        }
      }
      // -TODO: 前着地位置が0.037だった場合の処理を書くべき。遊脚Y軸のwalking_step-1を含む式の解が好ましくないものになる。
        // 歩行周期の前半を、重心位置を無視して遊脚Y軸の値を変更しないようにすれば良いはず。
        // -TODO: もっとキレイにできるはず。歩行開始時の歩行周期の最後に得られた遊脚軌道のY軸を記録しておいて、そこよりも０に近い値を取らないようにすればいい。下限を設定してやればいい。
        // TODO: ココと下の分岐は１つにまとめるべき。walking_step-1=0か、それ以外かになっている。参照するやつを判定して変数にwalking_step+-1を入れてやれば、１つにまとまる。
      else if(LandingPosition_[walking_step-1][2] == 0.037) {
        Foot_3D_Pos = {
          LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],  // x 
          (LandingPosition_[walking_step][2]-LandingPosition_[0][2])-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),  // y (基準点を右足接地点から胴体真下にするために、-0.037). 現着地位置ー現重心位置、Xなら脚を前から後ろに出すイメージ。
          -length_leg_  // z 
        };
        Foot_3D_Pos_Swing = {
          LandingPosition_[walking_step-1][1]+((LandingPosition_[walking_step+1][1]-LandingPosition_[walking_step-1][1])*(t/T_sup)),  // 前FP+(次FP-前FP)*t/Tsup
          (LandingPosition_[walking_step+1][2]-LandingPosition_[0][2])+(((LandingPosition_[walking_step+1][2]-LandingPosition_[0][2])-(LandingPosition_[walking_step+1][2]-LandingPosition_[0][2]))*(t/T_sup)) - (CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),  // 次FP+(次FP-次FP)*t/Tsup - 重心位置
          -length_leg_ + swing_trajectory // z (遊脚軌道をzから引く) 
        };
      }
      // -TODO: 次着地位置が0.037だった場合の処理を書くべき。遊脚Y軸のwalking_step-1を含む式の解が好ましくないものになる。
        // 次着地位置を参照せずに、前着地位置と次着地位置の差分を０と扱えばいい。つまり、walking_step+1をwalking_step-1に変えれば良いだけなはず。
        // -TODO: もっとキレイにできるはず。
      else if(LandingPosition_[walking_step+1][2] == 0.037) {
        Foot_3D_Pos = {
          LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],  // x 
          (LandingPosition_[walking_step][2]-LandingPosition_[0][2])-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),  // y (基準点を右足接地点から胴体真下にするために、-0.037). 現着地位置ー現重心位置、Xなら脚を前から後ろに出すイメージ。
          -length_leg_  // z 
        };
        Foot_3D_Pos_Swing = {
          LandingPosition_[walking_step-1][1]+((LandingPosition_[walking_step+1][1]-LandingPosition_[walking_step-1][1])*(t/T_sup)),  // 前FP+(次FP-前FP)*t/Tsup
          (LandingPosition_[walking_step-1][2]-LandingPosition_[0][2])+(((LandingPosition_[walking_step-1][2]-LandingPosition_[0][2])-(LandingPosition_[walking_step-1][2]-LandingPosition_[0][2]))*(t/T_sup)) - (CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),  // 前FP+(前FP-前FP)*t/Tsup - 重心位置
          -length_leg_ + swing_trajectory // z (遊脚軌道をzから引く) 
        };
      }
      else {  // 片脚支持。
        Foot_3D_Pos = {
          LandingPosition_[walking_step][1]-CoG_2D_Pos_local[control_step][0],  // x 
          (LandingPosition_[walking_step][2]-LandingPosition_[0][2])-(CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),  // y (基準点を右足接地点から胴体真下にするために、-0.037). 現着地位置ー現重心位置、Xなら脚を前から後ろに出すイメージ。
          -length_leg_  // z 
        };
        // TODO: 配列の外を参照する場合の処理を書く。walking_step-1とかwalking_step+1とか。
        Foot_3D_Pos_Swing = {
          LandingPosition_[walking_step-1][1]+((LandingPosition_[walking_step+1][1]-LandingPosition_[walking_step-1][1])*(t/T_sup)),  // 前FP+(次FP-前FP)*t/Tsup
          (LandingPosition_[walking_step-1][2]-LandingPosition_[0][2])+(((LandingPosition_[walking_step+1][2]-LandingPosition_[0][2])-(LandingPosition_[walking_step-1][2]-LandingPosition_[0][2]))*(t/T_sup)) - (CoG_2D_Pos_local[control_step][1]-LandingPosition_[0][2]),  // 前FP+(次FP-前FP)*t/Tsup - 重心位置
          -length_leg_ + swing_trajectory // z (遊脚軌道をzから引く) 
        };
      }

      // DEBUG: Logの吐き出し
      WPG_log_FootTrajectory << CoG_2D_Pos_world[control_step][0] << " " << CoG_2D_Pos_world[control_step][1]-LandingPosition_[0][2] << " " << Foot_3D_Pos.transpose() << " " << Foot_3D_Pos_Swing.transpose() << std::endl;

      // 重心速度の定義
      CoG_3D_Vel = {
        CoG_2D_Vel[control_step][0],  // liner x
        CoG_2D_Vel[control_step][1],  // liner y
        0,  // liner z
        0,  // rotation x
        0,  // rotation y
        0   // rotation z
      };

      // 支持脚の判定
      // 歩行開始、終了時
      // TODO: 今は、始まりの支持脚と終わりの支持脚が同じだからコレでOK。異なった場合も書くべき。
      if(LandingPosition_[walking_step][2] == 0.037) {
        // 両脚支持。遊脚はないので、左右どちらも重心位置からIKを解く。
// 
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

        // Jacobianの計算、Jacobianを記憶するクラス変数の更新
        JacobiMatrix_leg(Q_legR_, Q_legL_);

        // 各関節速度の計算
        // TODO: 足先速度も欲しいところ。座標変換行列を掛けていくだけで済む。
        jointVel_legR = Jacobi_legR_.inverse()*CoG_3D_Vel;
        jointVel_legL = Jacobi_legL_.inverse()*CoG_3D_Vel;

        // 歩行パラメータの代入
        WalkingPattern_Pos_legR_.push_back(Q_legR_);
        WalkingPattern_Vel_legR_.push_back({jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]});  // eigen::vectorをstd::arrayに変換するためにこうしている。
        WalkingPattern_Pos_legL_.push_back(Q_legL_);
        WalkingPattern_Vel_legL_.push_back({jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]});
      }
      // 左脚支持期
      else if(LandingPosition_[walking_step][2] > 0.037) {
        // 左脚支持。右脚遊脚。

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

        // Jacobianの計算、Jacobianを記憶するクラス変数の更新
        JacobiMatrix_leg(Q_legR_, Q_legL_);

        // 各関節速度の計算
        jointVel_legR = Jacobi_legR_.inverse()*CoG_3D_Vel;
        jointVel_legL = Jacobi_legL_.inverse()*CoG_3D_Vel;

        // 歩行パラメータの代入
        WalkingPattern_Pos_legR_.push_back(Q_legR_);  // 遊脚
        WalkingPattern_Vel_legR_.push_back({jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]});
        WalkingPattern_Pos_legL_.push_back(Q_legL_);  // 支持脚
        WalkingPattern_Vel_legL_.push_back({jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]});
      }
      // 右脚支持期
      else if(LandingPosition_[walking_step][2] < 0.037) {
        // 右脚支持。左脚遊脚。

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

        // Jacobianの計算、Jacobianを記憶するクラス変数の更新
        JacobiMatrix_leg(Q_legR_, Q_legL_);

        // 各関節速度の計算
        jointVel_legR = Jacobi_legR_.inverse()*CoG_3D_Vel;
        jointVel_legL = Jacobi_legL_.inverse()*CoG_3D_Vel;

        // 歩行パラメータの代入
        WalkingPattern_Pos_legR_.push_back(Q_legR_);  // 支持脚
        WalkingPattern_Vel_legR_.push_back({jointVel_legR[0], jointVel_legR[1], jointVel_legR[2], jointVel_legR[3], jointVel_legR[4], jointVel_legR[5]});
        WalkingPattern_Pos_legL_.push_back(Q_legL_);  // 遊脚
        WalkingPattern_Vel_legL_.push_back({jointVel_legL[0], jointVel_legL[1], jointVel_legL[2], jointVel_legL[3], jointVel_legL[4], jointVel_legL[5]});
      }

      // 更新
      control_step++;
      t += control_cycle;
      walking_time += control_cycle;

    }
    
    // DEBUG: Log file close
    WPG_log_WalkingPttern.close();

  }

  // マネージャからのCallback関数
  void WebotsRobotHandler::ControlOutput_Callback(const msgs_package::msg::ControlOutput::SharedPtr callback_data) {
    // RCLCPP_INFO(node_->get_logger(), "subscribe...: [ %d ]", callback_data->counter);
    (void)callback_data;
  }

  void WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    node_ = node;  // 他関数内でも使うため
    (void)parameters;  // fake

    auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

    using namespace std::placeholders;

    pub_feedback_ = node_->create_publisher<msgs_package::msg::Feedback>("Feedback", custom_QoS);
    sub_control_output_ = node_->create_subscription<msgs_package::msg::ControlOutput>("ControlOutput", custom_QoS, std::bind(&WebotsRobotHandler::ControlOutput_Callback, this, _1));


//  DEBUG: ===========================
    // DEBUG parameter setting
    DEBUG_ParameterSetting();

    // TODO: 歩行パターンを生成する
    WalkingPatternGenerate();

    // get motor tags & position_sensor tags
    for(int tag = 0; tag < 20; tag++) {  
      motorsTag_[tag] = wb_robot_get_device(motors_name_[tag].c_str());
      positionSensorsTag_[tag] = wb_robot_get_device((motors_name_[tag]+"S").c_str());
      wb_position_sensor_enable(positionSensorsTag_[tag], 1);  // enable & sampling_period: 100[ms]
    }
    accelerometerTag_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometerTag_, 1);  // enable & sampling_period: 100[ms]
    gyroTag_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyroTag_, 1);  // enable & sampling_period: 100[ms]

    // set init position & value
    // TODO: 脚の初期姿勢（特に位置）はIKの解から与えたい。今は角度を決め打ちで与えているので、初期姿勢の変更がめっちゃめんどくさい。
    // jointNum_legR_とかを使って、ここでIKを解いてinitJointAng_の指定列に結果を代入すればOK
    for(int tag = 0; tag < 20; tag++) {  
      getJointAng_[tag] = 0;
      wb_motor_set_position(motorsTag_[tag], initJointAng_[tag]);
      wb_motor_set_velocity(motorsTag_[tag], initJointVel_[tag]);
    }
    
    // DEBUG:
    wait_step = 500;
  }

  void WebotsRobotHandler::step() {
    // RCLCPP_INFO(node_->get_logger(), "step...");

    // TODO: 脚、腕と、専用の配列に入れ直すのだから、getJointAng_はもっと最適化できるはず。
    // get current status 
    // for(int tag = 0; tag < 20; tag++) {
    //   getJointAng_[tag] = wb_position_sensor_get_value(positionSensorsTag_[tag]);
    // }
    // accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
    // gyroValue_ = wb_gyro_get_values(gyroTag_);
    // // get leg_joints angle
    // for(int tag = 0; tag < 6; tag++) {
    //   Q_legR_[tag] = getJointAng_[jointNum_legR_[tag]];
    //   Q_legL_[tag] = getJointAng_[jointNum_legL_[tag]];
    // }
    // DEBUG:
    // auto hoge = FK_.getFK(Q_legR_, P_legR_waist_standard_, 6);
    // std::cout << hoge << std::endl;

    // DEBUG: 初期姿勢が完了するまでwait
    if(wait_step != 0) {
      wait_step--;
    }
    else if(wait_step == 0) {
      // DEBUG:
      if(control_step <= int(WalkingPattern_Pos_legR_.size()-1)) {

        // CHECKME: setするコードを書き直す。
        // set joints angle & velocity
        for(int tag = 0; tag < 6; tag++) {
          wb_motor_set_position(motorsTag_[jointNum_legR_[tag]], WalkingPattern_Pos_legR_[control_step][tag]*jointAng_posi_or_nega_legR_[tag]);
          wb_motor_set_velocity(motorsTag_[jointNum_legR_[tag]], std::abs(WalkingPattern_Vel_legR_[control_step][tag]));  // マイナスだと怒られるので、絶対値を取る
          wb_motor_set_position(motorsTag_[jointNum_legL_[tag]], WalkingPattern_Pos_legL_[control_step][tag]*jointAng_posi_or_nega_legL_[tag]);
          wb_motor_set_velocity(motorsTag_[jointNum_legL_[tag]], std::abs(WalkingPattern_Vel_legL_[control_step][tag]));
        }

        // // CHECKME: 読んだ歩行パターンを削除
        // WalkingPattern_Pos_legR_.erase(WalkingPattern_Pos_legR_.begin());  // CHECKME: 始端の削除。.begin()のほうが可読性が高いと思う。
        // WalkingPattern_Vel_legR_.erase(WalkingPattern_Vel_legR_.begin());  
        // WalkingPattern_Pos_legL_.erase(WalkingPattern_Pos_legL_.begin()); 
        // WalkingPattern_Vel_legL_.erase(WalkingPattern_Vel_legL_.begin()); 

      }

      control_step++;  // DEBUG: 
    }
  }

}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)