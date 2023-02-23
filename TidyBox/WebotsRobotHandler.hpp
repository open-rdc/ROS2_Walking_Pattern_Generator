#ifndef WEBOTS_ROBOT_HANDLER_HPP
#define WEBOTS_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
// #include "/*PackageName*//msg/ToWebotsRobotHandler_msgs.msg"

namespace webots_robot_handler
{
  class WebotsRobotHandler : public webots_ros2_driver::PluginInterface {
    public:
      void init(
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
      ) override;

      void step() override;

      void callback_res(const rclcpp::Client</*PackageName*/::srv::ToWebotsRobotHandler_msgs>::SharedFuture future);


    private:
      rclrpp::Client</*PackageName*/::msg::ToWebotsRobotHandler_msgs>::SharedPtr toWRH_clnt_ptr;
      
      // WbDeviceTagで宣言して、getDeviceするのはcppでって感じ？
      WbDeviceTag motor[20];
      WbDeviceTag positionSensor{20};
      WbDeviceTag gyro;
      WbDeviceTag accelerometer;

      float motorValue[20];
      float positionSensorValue[20];
      const double *accelerometerValue;
      const double *gyroValue;

  }
}

#endif