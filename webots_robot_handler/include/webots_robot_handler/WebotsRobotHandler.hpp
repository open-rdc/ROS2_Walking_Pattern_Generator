#ifndef WEBOTS_ROBOT_HANDLER_HPP
#define WEBOTS_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"

namespace webots_robot_handler
{
  class WebotsRobotHandler : public webots_ros2_driver::PluginInterface {
    public:
      void init(
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
      ) override;

      void step() override;

      void callback_res(const rclcpp::Client<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedFuture future);


    private:
      webots_ros2_driver::WebotsNode *node_;

      rclcpp::Client<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedPtr toWRH_clnt_;
      
      // Webots内のロボットが持つデバイスのタグを持つ。このタグをもとに、Webotsの関数はデバイスを区別する。
      WbDeviceTag motorsTag_[20];  // 全モータ２０個
      WbDeviceTag positionSensorsTag_[20];  // 全モータの回転角度センサ２０個
      WbDeviceTag gyroTag_;  // ジャイロセンサ
      WbDeviceTag accelerometerTag_;  // 加速度センサ

      double getJointAng_[20];  // Webots側から得た関節角度を記憶
      const double *accelerometerValue_;  
      const double *gyroValue_;

      // 処理に役立つ配列
      std::array<int, 6> jointNum_legR_;  // motorsTag[20]とpositionSensorTag[20]に対応する、モータ（とセンサ）の列番号を記憶（右足）
      std::array<int, 6> jointNum_legL_;  // 上に同じ（左足）
      std::array<int, 6> jointAng_posi_or_nega_legR_;  // モータの回転方向の系が、モータごとに違う。Kinematicsの方ではすべて右手系で計算している。ので、Webots内環境に合わせるための補正（正負の逆転）をかける。（右足）
      std::array<int, 6> jointAng_posi_or_nega_legL_;  // 上に同じ（左足）

      // DEBUG
      int count = 0;
// DEBUG===/*
      void DEBUG_ParameterSetting(void);

      std::array<std::string, 20> motors_name_;
      std::array<double, 20> initJointAng_;
      std::array<double, 20> initJointVel_;
// DEBUG===*/
  };
}

#endif