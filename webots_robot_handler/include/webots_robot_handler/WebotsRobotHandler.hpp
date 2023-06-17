#ifndef WEBOTS_ROBOT_HANDLER_HPP
#define WEBOTS_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/feedback.hpp"
#include "msgs_package/msg/control_output.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "Eigen/Dense"
#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"

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

      // Kinematicsライブラリの中に含めるべき関数
      void JacobiMatrix_leg(std::array<double, 6> Q_legR, std::array<double, 6> Q_legL);
      // マネージャからのCallback
      void ControlOutput_Callback(const msgs_package::msg::ControlOutput::SharedPtr callback_data);

// DEBUG: Dynamic Gait ==
      void WalkingPatternGenerate(void);

      // 共有ライブラリの実体化
      kinematics::FK FK_;
      kinematics::IK IK_;

      // TODO: Parameterから読み取るべき
      // TODO: 生成に必要な変数
      float weight_;
      float length_leg_;

      // TODO: 歩行パラメータを設定すべき
      // 歩行パラメータの行列
      // ex.
      // [[time, x_point, y_point]
      //  [time, x_point, y_point]
      //            ...
      //  [time, x_point, y_point]]
      std::vector<std::array<double, 3>> LandingPosition_;

      // 歩行パターンの変数（行列）
      std::vector<std::array<double, 6>> WalkingPattern_Pos_legR_;
      std::vector<std::array<double, 6>> WalkingPattern_Vel_legR_;
      std::vector<std::array<double, 6>> WalkingPattern_Pos_legL_;
      std::vector<std::array<double, 6>> WalkingPattern_Vel_legL_;

      // ヤコビアンとかに必要な変数
      Eigen::Matrix<double, 6, 6> Jacobi_legR_;
      Eigen::Matrix<double, 6, 6> Jacobi_legL_;
      // 以下、いる？
      std::array<Eigen::Vector3d, 6> P_FK_legR_;
      std::array<Eigen::Vector3d, 6> P_FK_legL_;
      std::array<Eigen::Vector3d, 6> UnitVec_legR_;
      std::array<Eigen::Vector3d, 6> UnitVec_legL_;

      std::array<Eigen::Matrix3d, 6> R_legR_;
      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Matrix3d, 6> R_legL_;
      std::array<Eigen::Vector3d, 7> P_legL_;

// == init() ==

      // init関数以外でもrclcpp::Nodeを使えるようにするため。
      webots_ros2_driver::WebotsNode *node_;

      rclcpp::Publisher<msgs_package::msg::Feedback>::SharedPtr pub_feedback_;
      rclcpp::Subscription<msgs_package::msg::ControlOutput>::SharedPtr sub_control_output_;
      
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

      // Parameter serverやURDF、Protoから読み込みたい。
      std::array<std::string, 20> motors_name_;
      std::array<double, 20> initJointAng_;
      std::array<double, 20> initJointVel_;


// == step() ==

      double getJointAng_[20];  // Webots側から得た関節角度を記憶
      const double *accelerometerValue_;  
      const double *gyroValue_;

      std::array<double, 6> Q_legR_;
      std::array<double, 6> Q_legL_;

  };
}

#endif