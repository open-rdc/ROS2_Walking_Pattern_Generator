#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include <chrono>
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
/*
・・・　cのヘッダでも、cppのヘッダでも、どっちでも良いと思う。
        ヘッダファイルのほうでは、WbDeviceTagで宣言して、getDeviceするのはこっちって感じ？
*/
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace webots_robot_handler
{
  void WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start up WebotsRobotHandler. Hello WebotsRobotHandler!!");

    toWRH_clnt = node->create_client<msgs_package::srv::ToWebotsRobotHandlerMessage>("FB_StabilizationController");

    while(!toWRH_clnt->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR!!: FB_StabilizationController service is dead.");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "Waiting for FB_StabilizationController service...");
    }

    auto toWRH_req = std::make_shared<msgs_package::srv::ToWebotsRobotHandlerMessage::Request>();

    std::array<const char*, 20> motors_name = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"), ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"), ("Neck"), ("Head")};

    char* S = "S";
    for(int i = 0; i < 20; i++) {
      motor[i] = wb_robot_get_device(motors_name[i]);
      positionSensor[i] = wb_robot_get_device(motors_name[i] + *S);
      wb_position_sensor_enable(positionSensor[i], 100);
    }
    accelerometer = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometer, 100);
    gyro = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyro, 100);
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