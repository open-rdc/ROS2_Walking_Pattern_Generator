#ifndef WEBOTS_ROBOT_HANDLER_HPP
#define WEBOTS_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"

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
      rclrpp::Client</*SrvMsg*/>::SharedPtr clnt_ptr;
      
      WbDeviceTag gyro;
      WbDeviceTag anklereg;  // WbDeviceTagで宣言して、getDeviceするのはcppでって感じ？

  }
}

#endif