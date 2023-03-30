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
      
      // WbDeviceTagで宣言して、getDeviceするのはcppでって感じ？
      WbDeviceTag motorsTag_[20];
      WbDeviceTag positionSensorsTag_[20];
      WbDeviceTag gyroTag_;
      WbDeviceTag accelerometerTag_;

      double setJointAng_[20];  // いる？
      double setJointVel_[20];
      double getJointAng_[20];
      const double *accelerometerValue_;
      const double *gyroValue_;

      // std::array<std::string, 20> motors_name;
      std::array<int, 6> jointNum_legR_;
      std::array<int, 6> jointNum_legL_;
      std::array<int, 6> jointAng_posi_or_nega_legR_;
      std::array<int, 6> jointAng_posi_or_nega_legL_;

      // DEBUG
      int count = 0;
  };
}

#endif