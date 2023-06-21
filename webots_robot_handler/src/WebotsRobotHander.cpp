#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
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

    // 単位ベクトルの設定。これもParameterやURDF, Protoから得たい。
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

    // Dynamic Gait ====
    weight_ = 3.0;  // [kg]
    length_leg_ = 219.5;  // [mm]
    // TODO: 足踏み。歩行する着地位置を計算して適用すべき
    LandingPosition_ = {{0.0, 0.0, -0.037},  // 歩行パラメータからの着地位置(time, x, y)
                        {0.8, 0.0, 0.037},
                        {1.6, 0.0, -0.037},
                        {2.4, 0.0, 0.037},
                        {3.2, 0.0, -0.037},
                        {4.0, 0.0, 0.037}};
  }

  // TODO: kinematics node でも作って、共有ライブラリにFK・IKともに入れたほうが良いと思う。
  void WebotsRobotHandler::JacobiMatrix_leg(std::array<double, 6> Q_legR, std::array<double, 6> Q_legL) {
    Jacobi_legR_ = MatrixXd::Zero(6, UnitVec_legR_.max_size());
    Jacobi_legL_ = MatrixXd::Zero(6, UnitVec_legR_.max_size());

//     // -TODO: ココは書き換える必要がある。
// // ココから
//     auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage::Request>();

//     toKine_FK_req->q_target_r = Q_legR;
//     toKine_FK_req->q_target_l = Q_legL;

        // CHECKME: 確認すべき
        for(int tag = 0; tag < int(UnitVec_legR_.max_size()); tag++) {
          P_FK_legR_[tag] = FK_.getFK(Q_legR, P_legR_waist_standard_, tag);
          P_FK_legL_[tag] = FK_.getFK(Q_legL, P_legL_waist_standard_, tag);
        }

//     for(int i = 0; i < int(UnitVec_legR_.max_size()); i++) {
//       toKine_FK_req->fk_point = i; 

//       auto toKine_FK_res = toKine_FK_clnt_->async_send_request(
//         toKine_FK_req, 
//         [this, i](const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future) {
//           P_FK_legR_[i] = {future.get()->p_result_r[0], future.get()->p_result_r[1], future.get()->p_result_r[2]};
//           P_FK_legL_[i] = {future.get()->p_result_l[0], future.get()->p_result_l[1], future.get()->p_result_l[2]};
//         }
//       );
//       rclcpp::spin_until_future_complete(this->get_node_base_interface(), toKine_FK_res);

//       // std::cout << i << std::endl;
//       // std::cout << "legR: " << P_FK_legR_[i].transpose() << std::endl;
//       // std::cout << "legL: " << P_FK_legL_[i].transpose() << std::endl;
//     }
//     std::cout << std::endl;
// // ココまで

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
        // std::cout << "pt_P_legR: " << pt_P_legR.transpose() << std::endl;
        // std::cout << "pt_P_legL: " << pt_P_legL.transpose() << std::endl;
        mat_legR = UnitVec_legR_[tag].cross(pt_P_legR);
        mat_legL = UnitVec_legL_[tag].cross(pt_P_legL);
      }

      // 異常値への対処
      for(int i = 0; i < 3; i++) {
        if(abs(mat_legR[i]) < 0.000001) {
          mat_legR[i] = 0;
        }
        if(abs(mat_legL[i]) < 0.000001) {
          mat_legL[i] = 0;
        }
        if(abs(mat_legR[i]) > 10000000) {
          mat_legR[i] = 0;
        }
        if(abs(mat_legL[i]) > 10000000) {
          mat_legL[i] = 0;
        }
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
    // flow
    // 1. loop(0.01[s]刻み、数[s])
    // 2. 重心位置計算
    // 3. if(支持脚切り替えタイミングか否か)
    //   4. True: 支持脚切り替え・P_leg切り替え、歩行素片初期重心位置でIKを解いて歩行パターンへpushback
    //   5. False: 重心位置でIKを解いて歩行パターンへpushback

    // 制御周期
    float control_cycle = 0.01;  // [s]

    // 歩行パラメータの最終着地時間[s]を抽出
    float walking_time_max = LandingPosition_[LandingPosition_.max_size()-1][0];  // TODO: 無駄な変数なので消すべき。わかりやすさ重視 

    // 着地位置修正の最適化での重み
    int opt_weight_pos = 10;
    int opt_weight_vel = 1;

    // 重心位置・速度を保持する変数（重心は腰に位置するものとする）
    std::vector<std::array<double, 6>> CoG_2D_Pos;  // {{x0,y0},{x1,y1},{x2,y2}}
    std::vector<std::array<double, 6>> CoG_2D_Vec;

    // 修正着地位置（ｘ）
    double p_x_fix;

    // 時間, 時定数
    float T_sup = 0;  // 0 ~ 支持脚切り替え時間
    float T_sup_max = LandingPosition_[1][0];  // 0.8. 支持脚切り替えタイミング. 歩行素片終端時間
    float T_c = std::sqrt(length_leg_ / 9.81);  // 時定数

    // loop. 0: control_cycle: walking_time_max
    int control_step = 0;
    int walking_step = 0;
    float walking_time = 0;
    double S, C;
    while(walking_time <= walking_time_max) {
      // 行を追加
      CoG_2D_Pos.push_back({0, 0});
      CoG_2D_Vec.push_back({0, 0});

      // sinh(Tsup/Tc), cosh(Tsup/Tc)
      S = std::sinh(T_sup / T_c);
      C = std::cosh(T_sup / T_c);
      
      // 重心位置の計算
      CoG_2D_Pos[control_step][0] = 0;  // position_x
      CoG_2D_Pos[control_step][1] = 0;  // position_y
      // 重心速度の計算
      CoG_2D_Vec[control_step][0] = 0;
      CoG_2D_Vec[control_step][1] = 0;

      // 支持脚切り替えの判定
      if(T_sup == T_sup_max) {
        // 着地位置の取得（後で修正着地位置が代入される）
        p_x_fix = LandingPosition_[walking_step][1];
        
        // 値の更新
        T_sup = 0;
        walking_step++;
      }
      else {
        // 値の更新
        T_sup += control_cycle;
      }

      // 値の更新
      control_step++;
      walking_time += control_cycle;
    }

    // 歩行パラメータの定義
    WalkingPattern_Pos_legR_.push_back({0, 0, 0, 0, 0, 0});  // CHECKME: 歩行パターンの行列に１ステップ分を末端に追加
    WalkingPattern_Vel_legR_.push_back({0, 0, 0, 0, 0, 0});
    WalkingPattern_Pos_legL_.push_back({0, 0, 0, 0, 0, 0});
    WalkingPattern_Vel_legL_.push_back({0, 0, 0, 0, 0, 0});
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



    for(int tag = 0; tag < 20; tag++) {  // get motor tags & position_sensor tags
      motorsTag_[tag] = wb_robot_get_device(motors_name_[tag].c_str());
      positionSensorsTag_[tag] = wb_robot_get_device((motors_name_[tag]+"S").c_str());
      wb_position_sensor_enable(positionSensorsTag_[tag], 1);  // enable & sampling_period: 100[ms]
    }
    accelerometerTag_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometerTag_, 1);  // enable & sampling_period: 100[ms]
    gyroTag_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyroTag_, 1);  // enable & sampling_period: 100[ms]

    // RCLCPP_INFO(node_->get_logger(), "Set init joints_angle.");
    for(int tag = 0; tag < 20; tag++) {  // set init position & value
      getJointAng_[tag] = 0;
      wb_motor_set_position(motorsTag_[tag], initJointAng_[tag]);
      wb_motor_set_velocity(motorsTag_[tag], initJointVel_[tag]);
    }
    
  }

  void WebotsRobotHandler::step() {
    // RCLCPP_INFO(node_->get_logger(), "step...");

    // TODO: 脚、腕と、専用の配列に入れ直すのだから、getJointAng_はもっと最適化できるはず。
    // get current status 
    for(int tag = 0; tag < 20; tag++) {
      getJointAng_[tag] = wb_position_sensor_get_value(positionSensorsTag_[tag]);
    }
    accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
    gyroValue_ = wb_gyro_get_values(gyroTag_);
    // get leg_joints angle
    // for(int tag = 0; tag < 6; tag++) {
    //   Q_legR_[tag] = getJointAng_[jointNum_legR_[tag]];
    //   Q_legL_[tag] = getJointAng_[jointNum_legL_[tag]];
    // }

    // CHECKME: setするコードを書き直す。
    // set joints angle & velocity
    for(int tag = 0; tag < 6; tag++) {
      wb_motor_set_position(motorsTag_[jointNum_legR_[tag]], WalkingPattern_Pos_legR_[0][tag]*jointAng_posi_or_nega_legR_[tag]);
      wb_motor_set_velocity(motorsTag_[jointNum_legR_[tag]], WalkingPattern_Vel_legR_[0][tag]);
      wb_motor_set_position(motorsTag_[jointNum_legL_[tag]], WalkingPattern_Pos_legL_[0][tag]*jointAng_posi_or_nega_legL_[tag]);
      wb_motor_set_velocity(motorsTag_[jointNum_legL_[tag]], WalkingPattern_Vel_legL_[0][tag]);
    }

    // CHECKME: 読んだ歩行パターンを削除
    WalkingPattern_Pos_legR_.erase(WalkingPattern_Pos_legR_.begin());  // CHECKME: 始端の削除。.begin()のほうが可読性が高いと思う。
    WalkingPattern_Vel_legR_.erase(WalkingPattern_Vel_legR_.begin());  
    WalkingPattern_Pos_legL_.erase(WalkingPattern_Pos_legL_.begin()); 
    WalkingPattern_Vel_legL_.erase(WalkingPattern_Vel_legL_.begin()); 
  }

}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)