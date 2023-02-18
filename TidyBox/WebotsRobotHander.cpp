#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
/*
#include <webots/robot.h>
#include <webots/motor.h>
・・・　cのヘッダでも、cppのヘッダでも、どっちでも良いと思う。
        ヘッダファイルのほうでは、WbDeviceTagで宣言して、getDeviceするのはこっちって感じ？
*/

namespace webots_robot_handler
{

}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)