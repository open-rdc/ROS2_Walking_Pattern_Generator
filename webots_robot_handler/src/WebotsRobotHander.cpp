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



using namespace std::chrono_literals;
using namespace std::placeholders;



namespace webots_robot_handler
{
  void WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    node_ = node;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start up WebotsRobotHandler. Hello WebotsRobotHandler!!");

    toWRH_clnt_ = node_->create_client<msgs_package::srv::ToWebotsRobotHandlerMessage>("FB_StabilizationController");

    // while(!toWRH_clnt_->wait_for_service(1s)) {
    //   if(!rclcpp::ok()) {
    //     RCLCPP_ERROR(node_->get_logger(), "ERROR!!: FB_StabilizationController service is dead.");
    //     return;
    //   }
    //   RCLCPP_INFO(node_->get_logger(), "Waiting for FB_StabilizationController service...");
    // }

    auto toWRH_req = std::make_shared<msgs_package::srv::ToWebotsRobotHandlerMessage::Request>();

    std::array<const char*, 20> motors_name = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"), ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"), ("Neck"), ("Head")};

    char* S = "S";
    for(int i = 0; i < 20; i++) {  // get motors & position_sensors
      motor_[i] = wb_robot_get_device(motors_name[i]);
      positionSensor_[i] = wb_robot_get_device(motors_name[i] + *S);
      wb_position_sensor_enable(positionSensor_[i], 100);
    }
    accelerometer_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometer_, 100);  // enable & sampling_period: 100[ms]
    gyro_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyro_, 100);

    RCLCPP_INFO(node_->get_logger(), "Set init joints_angle.");
    std::array<const double, 20> initJointAng = {0, 0, 0.79, -0.79, -1.57, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.26};  // init motors ang [rad] 
    for(int i = 0; i < 20; i++) {  // set init position & value
      setJointAng_[i] = 0;  // いる？
      setJointVel_[i] = 0;
      getJointAng_[i] = 0;
      wb_motor_set_position(motor_[i], initJointAng[i]);
      wb_motor_set_velocity(motor_[i], 0.25);
    }

    RCLCPP_INFO(node_->get_logger(), "Finish init, Start step.");
  }


  void WebotsRobotHandler::callback_res(
    rclcpp::Client<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedFuture future
  ) {
    // callback function
  }


  void WebotsRobotHandler::step() {
    RCLCPP_INFO(node_->get_logger(), "step...");

    for(int i = 0; i < 20; i++) {
      getJointAng_[i] = wb_position_sensor_get_value(positionSensor_[i]);
    }
    accelerometerValue_ = wb_accelerometer_get_values(accelerometer_);
    gyroValue_ = wb_gyro_get_values(gyro_);
  }
}



PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)