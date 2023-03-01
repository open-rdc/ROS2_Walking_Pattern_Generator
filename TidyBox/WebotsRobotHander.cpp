#include "Webots_Robot_Handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "Msgs_Package/msg/ToWebotsRobotHandler_msgs.msg"
/*
#include <webots/robot.h>
#include <webots/motor.h>
・・・　cのヘッダでも、cppのヘッダでも、どっちでも良いと思う。
        ヘッダファイルのほうでは、WbDeviceTagで宣言して、getDeviceするのはこっちって感じ？
*/

namespace webots_robot_handler
{
  void init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    // init
  }

  void callback_res(
    rclcpp::Client<Msgs_Package::msg::ToWebotsRobotHandler_msgs>::SharedFuture future
  ) {
    // callback function
  }

  void step() {
    // step
  }
}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)