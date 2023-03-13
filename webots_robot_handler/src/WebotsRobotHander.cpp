#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"
/*
#include <webots/robot.h>
#include <webots/motor.h>
・・・　cのヘッダでも、cppのヘッダでも、どっちでも良いと思う。
        ヘッダファイルのほうでは、WbDeviceTagで宣言して、getDeviceするのはこっちって感じ？
*/

namespace webots_robot_handler
{
  void WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    // init
  }

  void WebotsRobotHandler::callback_res(
    rclcpp::Client<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedFuture future
  ) {
    // callback function
  }

  void WebotsRobotHandler::step() {
    // step
  }
}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)