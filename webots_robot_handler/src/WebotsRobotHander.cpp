#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include <fstream>  // Logをファイルに吐くため
// #include <rmw/qos_profiles.h>
// #include "msgs_package/msg/control_output.hpp"
// #include "msgs_package/msg/feedback.hpp"
#include "robot_messages/msg/feedback.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>

using namespace std::chrono_literals;

namespace webots_robot_handler
{
  // static const rmw_qos_profile_t custom_qos_profile =
  // {
  //   RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
  //   1,  // History(keep_last) Depth
  //   RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
  //   RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
  //   RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
  //   RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
  //   RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
  //   RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
  //   false  // avoid_ros_namespace_conventions
  // };

  void WebotsRobotHandler::DEBUG_ParameterSetting() {
    // protoに沿った各モータの名前
    motors_name_ = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"),   // arm
                    ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"),   //leg
                    ("Neck"), ("Head")};  //body
    // 初期姿勢を設定. init joints ang [rad]. corresponding to motors_name
    // initJointAng_ = {0, 0, -0.5, 0.5, -1, 1,   // arm
    //                 0, 0, 0, 0, -3.14/8, 3.14/8, 3.14/4, -3.14/4, 3.14/8, -3.14/8, 0, 0,   // leg
    //                 0, 0.26};  // body
    initJointAng_ = {0, 0, 0, 0, 0, 0,   // arm
                    0, 0, 0, 0, -3.14/8, 3.14/8, 3.14/4, -3.14/4, 3.14/8, -3.14/8, 0, 0,   // leg
                    0, 0};  // body
        // initJointAng_ = {0, 0, -0.5, 0.5, -1, 1,   // arm
        //             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   // leg
        //             0, 0};  // body
    // 初期姿勢に移行する時の角速度.  init joints vel [rad/s]
    initJointVel_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5,  // arm
                    0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5,  // log
                    0.5, 0.5};  // body

    // 脚のラベル一覧
    jointNum_legR_ = {6, 8, 10, 12, 14, 16};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(right leg)
    jointNum_legL_ = {7, 9, 11, 13, 15, 17};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(left leg)
    // IKなどは右手系で解いているが、ロボットで左手系を採用している関節が幾つかある。それに対応する為の配列。
    // jointAng_posi_or_nega_legR_ = {-1, -1, 1, 1, -1, 1};  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
    // jointAng_posi_or_nega_legL_ = {-1, -1, -1, -1, 1, 1}; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
    jointAng_posi_or_nega_legR_ = {1, 1, 1, 1, 1, 1};  //
    jointAng_posi_or_nega_legL_ = {1, 1, 1, 1, 1, 1}; // URDF側で回転系の補間を行っている。ので、Handler側で行う必要はない。

    weight_ = 3.0;  // [kg]
    length_leg_ = 171.856 / 1000;  // [m] ちょっと中腰。特異点を回避。直立：219.5[mm]
  }

  // CHECKME: Rviz2との連携を確認するために、JointState TopicをSubscribe
  void WebotsRobotHandler::JointStates_Callback(const sensor_msgs::msg::JointState::SharedPtr callback_data) {
    // for(int i = 0; i < 20; i++) {
    //   std::cout << callback_data->name[i] << "   " << callback_data->position[i] << "   " << callback_data->velocity[i] << std::endl;
    // }
    // WalkingPattern_Pos_legL_.resize(1);
    // WalkingPattern_Pos_legR_.resize(1);
    // WalkingPattern_Vel_legL_.resize(1);
    // WalkingPattern_Vel_legR_.resize(1);
    WalkingPattern_Pos_legL_[0] = {
      callback_data->position[8],
      callback_data->position[9],
      callback_data->position[10],
      callback_data->position[11],
      callback_data->position[12],
      callback_data->position[13]
    };
    WalkingPattern_Pos_legR_[0] = {
      callback_data->position[14],
      callback_data->position[15],
      callback_data->position[16],
      callback_data->position[17],
      callback_data->position[18],
      callback_data->position[19]
    };
    WalkingPattern_Vel_legL_[0] = {
      callback_data->velocity[8],
      callback_data->velocity[9],
      callback_data->velocity[10],
      callback_data->velocity[11],
      callback_data->velocity[12],
      callback_data->velocity[13]
    };
    WalkingPattern_Vel_legR_[0] = {
      callback_data->velocity[14],
      callback_data->velocity[15],
      callback_data->velocity[16],
      callback_data->velocity[17],
      callback_data->velocity[18],
      callback_data->velocity[19]
    };
  }

  void WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    node_ = node;  // 他関数内でも使うため
    (void)parameters;  // fake

    // auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

    using namespace std::placeholders;

    // CHECKME
    sub_joint_state_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&WebotsRobotHandler::JointStates_Callback, this, _1));
    pub_feedback_ = node_->create_publisher<robot_messages::msg::Feedback>("feedback", 10);

    // DEBUG: parameter setting
    DEBUG_ParameterSetting();

    // get motor tags & position_sensor tags
    for(int tag = 0; tag < 20; tag++) {  
      motorsTag_[tag] = wb_robot_get_device(motors_name_[tag].c_str());
      positionSensorsTag_[tag] = wb_robot_get_device((motors_name_[tag]+"S").c_str());
      wb_position_sensor_enable(positionSensorsTag_[tag], 1);  // enable & sampling_period: 1[ms]
    }
    accelerometerTag_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometerTag_, 1);  // enable & sampling_period: 1[ms]
    gyroTag_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyroTag_, 1);  // enable & sampling_period: 1[ms]

    // set init position & value
    // TODO: 脚の初期姿勢（特に位置）はIKの解から与えたい。今は角度を決め打ちで与えているので、初期姿勢の変更がめっちゃめんどくさい。
    // jointNum_legR_とかを使って、ここでIKを解いてinitJointAng_の指定列に結果を代入すればOK
    for(int tag = 0; tag < 20; tag++) {  
      getJointAng_[tag] = 0;
      wb_motor_set_position(motorsTag_[tag], initJointAng_[tag]);
      wb_motor_set_velocity(motorsTag_[tag], initJointVel_[tag]);
    }
    
    // // DEBUG: 初期姿勢への移行が済むまで待つための変数。決め打ち
    // // TODO: IMUで初期姿勢への移行完了を検知したい。値がある一定値以内になったらフラグを解除する方式を取りたい
    wait_step = 500;  // 500 * 10[ms] = 5[s]
  }

  // 恐らく、.wbtのstep_timeを10[ms]に設定してても、step()が10[ms]以内で回る保証はないのだろう。
  void WebotsRobotHandler::step() {
    // RCLCPP_INFO(node_->get_logger(), "step...");
    // // DEBUG: 初期姿勢が完了するまでwait
    if(wait_step != 0) {
      wait_step--;
    }
    else {
/* Accelerometer & Gyro. Darwin-op.proto 仕様
source: https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto

        Accelerometer {
          translation -0.01 0 -0.068
          rotation 0 0 1.0 -1.5708  # z軸基準に座標を-90°回転 (x -> -y, y -> x, z -> z)
          name "Accelerometer"
          lookupTable [
            -39.24 0 0 39.24 1024 0
          ] 生のデータ：-39.24 ~ 39.24 [m/s^2] -> これを、0 ~ 1024にマッピング。
        }
        Gyro {
          translation 0.01 0 -0.068
          rotation 0 0 1.0 -3.1416  # z軸基準に座標を-180°回転 (x -> -x, y -> -y, z -> z)
          name "Gyro"
          lookupTable [
            -27.925 0 0 27.925 1024 0
          ]
        }

      Offset (センサデータ出力値より推測)
      Acce
        x: 512
        y: 512
        z: 640
      Gyro
        x: 512
        y: 512
        z: 512
        
reference:
  acce: https://github.com/cyberbotics/webots/blob/master/docs/reference/accelerometer.md
  gyro: https://github.com/cyberbotics/webots/blob/master/docs/reference/gyro.md
*/
      // feedback acce, gyro & joint_pos
      for(int tag = 0; tag < 6; tag++) {
        pub_feedback_msg_->q_now_leg_l[tag] = wb_position_sensor_get_value(positionSensorsTag_[jointNum_legL_[tag]]);
        pub_feedback_msg_->q_now_leg_r[tag] = wb_position_sensor_get_value(positionSensorsTag_[jointNum_legR_[tag]]);
      }
      accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
      gyroValue_ = wb_gyro_get_values(gyroTag_);

      // fixed offset & axis-pose
      pub_feedback_msg_->accelerometer_now[0] = accelerometerValue_[1];
      pub_feedback_msg_->accelerometer_now[1] = 1024-(accelerometerValue_[0]);
      pub_feedback_msg_->accelerometer_now[2] = accelerometerValue_[2];
      pub_feedback_msg_->gyro_now[0] = 1024-(gyroValue_[0]);
      pub_feedback_msg_->gyro_now[1] = 1024-(gyroValue_[1]);
      pub_feedback_msg_->gyro_now[2] = gyroValue_[2];
      pub_feedback_msg_->step_count = walking_step;

      walking_step++;

      for(int tag = 0; tag < 6; tag++) {
        wb_motor_set_position(motorsTag_[jointNum_legR_[tag]], WalkingPattern_Pos_legR_[0][tag]);  // DEBUG: control_Step -> 0
        wb_motor_set_velocity(motorsTag_[jointNum_legR_[tag]], WalkingPattern_Vel_legR_[0][tag]);
        wb_motor_set_position(motorsTag_[jointNum_legL_[tag]], WalkingPattern_Pos_legL_[0][tag]);
        wb_motor_set_velocity(motorsTag_[jointNum_legL_[tag]], WalkingPattern_Vel_legL_[0][tag]);
      }
    
      pub_feedback_->publish(*pub_feedback_msg_);
    }
  }

}

/*ERROR
[ERROR] [driver-3]: process has died [pid 31087, exit code -11, cmd '/opt/ros/humble/lib/webots_ros2_driver/driver --ros-args --params-file /tmp/launch_params_hyd9q3m9 --params-file /tmp/launch_params_avs0yvpn'].
*/

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)