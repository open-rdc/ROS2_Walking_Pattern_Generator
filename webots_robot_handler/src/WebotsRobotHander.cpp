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

using namespace std::chrono_literals;

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

    weight_ = 3.0;  // [kg]
    length_leg_ = 171.856 / 1000;  // [m] ちょっと中腰。特異点を回避。直立：219.5[mm]
  }

  // マネージャからのCallback関数
  // TODO: Pub/Subなので、データの受取ミスが稀に起きる。stackに余裕を持たせているから1stepの抜け程度なら今は大丈夫。だが、データ落ちは０にしたい。
  void WebotsRobotHandler::ControlOutput_Callback(const msgs_package::msg::ControlOutput::SharedPtr callback_data) {
    // RCLCPP_INFO(node_->get_logger(), "subscribe...: [ %d ]", callback_data->counter);
    WalkingPattern_Pos_legL_.push_back(callback_data->q_next_leg_l);
    WalkingPattern_Pos_legR_.push_back(callback_data->q_next_leg_r);
    WalkingPattern_Vel_legL_.push_back(callback_data->dq_next_leg_l);
    WalkingPattern_Vel_legR_.push_back(callback_data->dq_next_leg_r);

    // LOG: Pub/Subのデータ落ちを記録
    // int diff = callback_data->counter - counter_old_;
    // if(1 != diff) {
    //   loss_count_++;
    //   if(2 == diff) {
    //     RCLCPP_WARN(node_->get_logger(), "WalkingPattern Data Loss!!: loss count [ %d ], loss data step number [ %d ]", loss_count_, counter_old_+1);
    //   }
    //   else {
    //     for(int loss_step = 1; loss_step <= diff; loss_step++) {
    //       RCLCPP_WARN(node_->get_logger(), "WalkingPattern Data Loss!!: loss count [ %d ], loss data step number [ %d ]", loss_count_, counter_old_+loss_step);
    //     }
    //   }
    // }
    // counter_old_ = callback_data->counter;
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

    pub_feedback_msg_ = std::make_shared<msgs_package::msg::Feedback>();

    // DEBUG: parameter setting
    DEBUG_ParameterSetting();

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
    
    // DEBUG: 初期姿勢への移行が済むまで待つための変数。決め打ち
    // TODO: IMUで初期姿勢への移行完了を検知したい。値がある一定値以内になったらフラグを解除する方式を取りたい
    wait_step = 500;  // 500 * 10[ms] = 5[s]
  }

  // 恐らく、.wbtのstep_timeを10[ms]に設定してても、step()が10[ms]以内で回る保証はないのだろう。
  void WebotsRobotHandler::step() {
    // RCLCPP_INFO(node_->get_logger(), "step...");

    // TODO: 脚、腕と、専用の配列に入れ直すのだから、getJointAng_はもっと最適化できるはず。
    // get current status 
    // for(int tag : jointNum_legR_) {
    // for(int tag = 0; tag < int(jointNum_legR_.size()); tag++) {
    //   // getJointAng_[tag] = wb_position_sensor_get_value(positionSensorsTag_[tag]);
    //   Q_legR_[tag] =  wb_position_sensor_get_value(positionSensorsTag_[jointNum_legR_[tag]]);
    //   Q_legL_[tag] =  wb_position_sensor_get_value(positionSensorsTag_[jointNum_legL_[tag]]);
    // }
    // accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);  // TODO: 512基準の実数１つだけ。３軸全部getしたい。
    // gyroValue_ = wb_gyro_get_values(gyroTag_);  // TODO: 上に同じ。変数の型から変える必要がある。

    // pub_feedback_msg_->step_count = simu_step;
    // pub_feedback_msg_->q_now_leg_r = Q_legR_;
    // pub_feedback_msg_->q_now_leg_l = Q_legL_;
    // pub_feedback_msg_->accelerometer_now[0] = accelerometerValue_[0];
    // pub_feedback_msg_->accelerometer_now[1] = accelerometerValue_[1];
    // pub_feedback_msg_->accelerometer_now[2] = accelerometerValue_[2];
    // pub_feedback_msg_->gyro_now[0] = gyroValue_[0];
    // pub_feedback_msg_->gyro_now[1] = gyroValue_[1];
    // pub_feedback_msg_->gyro_now[2] = gyroValue_[2];

    // // publish feedback
    // pub_feedback_->publish(*pub_feedback_msg_);

    // get leg_joints angle
    // for(int tag = 0; tag < 6; tag++) {
    //   Q_legR_[tag] = getJointAng_[jointNum_legR_[tag]];
    //   Q_legL_[tag] = getJointAng_[jointNum_legL_[tag]];
    // }
    // -DEBUG:
    // auto hoge = FK_.getFK(Q_legR_, P_legR_waist_standard_, 6);
    // std::cout << hoge << std::endl;

    // DEBUG: 初期姿勢が完了するまでwait
    if(wait_step != 0) {
      wait_step--;
    }
    // DEBUG: 200は決め打ち。余裕があったほうがいいだろうという判断。
    else if((wait_step == 0)  && (200 < int(WalkingPattern_Pos_legL_.size()))) {
      for(int tag = 0; tag < int(jointNum_legR_.size()); tag++) {
        // getJointAng_[tag] = wb_position_sensor_get_value(positionSensorsTag_[tag]);
        Q_legR_[tag] =  wb_position_sensor_get_value(positionSensorsTag_[jointNum_legR_[tag]]);
        Q_legL_[tag] =  wb_position_sensor_get_value(positionSensorsTag_[jointNum_legL_[tag]]);
      }
      accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);  // TODO: 512基準の実数１つだけ。３軸全部getしたい。
      gyroValue_ = wb_gyro_get_values(gyroTag_);  // TODO: 上に同じ。変数の型から変える必要がある。

      pub_feedback_msg_->step_count = control_step;
      pub_feedback_msg_->q_now_leg_r = Q_legR_;
      pub_feedback_msg_->q_now_leg_l = Q_legL_;
      pub_feedback_msg_->accelerometer_now[0] = accelerometerValue_[0];
      pub_feedback_msg_->accelerometer_now[1] = accelerometerValue_[1];
      pub_feedback_msg_->accelerometer_now[2] = accelerometerValue_[2];
      pub_feedback_msg_->gyro_now[0] = gyroValue_[0];
      pub_feedback_msg_->gyro_now[1] = gyroValue_[1];
      pub_feedback_msg_->gyro_now[2] = gyroValue_[2];

      // publish feedback
      pub_feedback_->publish(*pub_feedback_msg_);

      // 歩行パターンが存在するか
      // TODO: control_stepは良くないのでは？
      if(control_step <= int(WalkingPattern_Pos_legR_.size()-1)) {
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
    // simu_step++;  // DEBUG:
  }

}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)