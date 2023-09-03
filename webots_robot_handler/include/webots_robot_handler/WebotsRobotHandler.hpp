#ifndef WEBOTS_ROBOT_HANDLER_HPP
#define WEBOTS_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/feedback.hpp"
#include "msgs_package/msg/control_output.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace webots_robot_handler
{
  class WebotsRobotHandler : public webots_ros2_driver::PluginInterface {
    public:
      void init(
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
      ) override;

      void step() override;

    private:
      // マネージャからのCallback
      // TODO: ここの型をJointStateにして、ros2_controlに対応させる。さすればRviz2との連携も可能。
      void ControlOutput_Callback(const msgs_package::msg::ControlOutput::SharedPtr callback_data);

      // CHECKME
      void JointStates_Callback(const sensor_msgs::msg::JointState::SharedPtr callbacl_data);

      // DEBUG: 1つ前のcounterを記憶。データ落ちが無いかの判定に用いる。
      int counter_old_ = -1;
      int loss_count_ = 0;

      // TODO: Parameterから読み取るべき
      // TODO: 生成に必要な変数
      float weight_;
      float length_leg_;

      // 歩行パターンの変数（行列）
      std::vector<std::array<double, 6>> WalkingPattern_Pos_legR_;
      std::vector<std::array<double, 6>> WalkingPattern_Vel_legR_;
      std::vector<std::array<double, 6>> WalkingPattern_Pos_legL_;
      std::vector<std::array<double, 6>> WalkingPattern_Vel_legL_;

      std::array<double, 6> Q_legR_;
      std::array<double, 6> Q_legL_;

// == init() ==

      // init関数以外でもrclcpp::Nodeを使えるようにするため。
      webots_ros2_driver::WebotsNode *node_;

      rclcpp::Publisher<msgs_package::msg::Feedback>::SharedPtr pub_feedback_;
      // TODO: ここの型をJointStateにして、ros2_controlに対応させる。さすればRviz2との連携も可能。
      rclcpp::Subscription<msgs_package::msg::ControlOutput>::SharedPtr sub_control_output_;

      // CHECKME
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

      std::shared_ptr<msgs_package::msg::Feedback> pub_feedback_msg_;
      
      // Webots内のロボットが持つデバイスのタグを持つ。このタグをもとに、Webotsの関数はデバイスを区別する。
      WbDeviceTag motorsTag_[20];  // 全モータ２０個
      WbDeviceTag positionSensorsTag_[20];  // 全モータの回転角度センサ２０個
      WbDeviceTag gyroTag_;  // ジャイロセンサ
      WbDeviceTag accelerometerTag_;  // 加速度センサ

      // 処理に役立つ配列
      std::array<int, 6> jointNum_legR_;  // motorsTag[20]とpositionSensorTag[20]に対応する、モータ（とセンサ）の列番号を記憶（右足）
      std::array<int, 6> jointNum_legL_;  // 上に同じ（左足）
      std::array<int, 6> jointAng_posi_or_nega_legR_;  // モータの回転方向の系が、モータごとに違う。Kinematicsの方ではすべて右手系で計算している。ので、Webots内環境に合わせるための補正（正負の逆転）をかける。（右足）
      std::array<int, 6> jointAng_posi_or_nega_legL_;  // 上に同じ（左足）

// DEBUG:===/*
      void DEBUG_ParameterSetting(void);

      // TODO: Parameter serverやURDF、Protoから読み込みたい。
      std::array<std::string, 20> motors_name_;
      std::array<double, 20> initJointAng_;
      std::array<double, 20> initJointVel_;


// == step() ==

      double getJointAng_[20];  // Webots側から得た関節角度を記憶
      const double *accelerometerValue_;  
      const double *gyroValue_;

      int wait_step;  // DEBUG: 初期姿勢になるまで待機するstep数

      int control_step;  // DEBUG: 

      int simu_step = 0;  // DEBUG:
  };
}

#endif