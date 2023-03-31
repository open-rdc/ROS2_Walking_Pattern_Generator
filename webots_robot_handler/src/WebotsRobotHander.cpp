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

    // check service server
    while(!toWRH_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "ERROR!!: FB_StabilizationController service is dead.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for FB_StabilizationController service...");
    }

    std::array<const std::string, 20> motors_name = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"), ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"), ("Neck"), ("Head")};

    for(int i = 0; i < 20; i++) {  // get motor tags & position_sensor tags
      motorsTag_[i] = wb_robot_get_device(motors_name[i].c_str());
      positionSensorsTag_[i] = wb_robot_get_device((motors_name[i]+"S").c_str());
      wb_position_sensor_enable(positionSensorsTag_[i], 100);
    }
    accelerometerTag_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometerTag_, 100);  // enable & sampling_period: 100[ms]
    gyroTag_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyroTag_, 100);

    RCLCPP_INFO(node_->get_logger(), "Set init joints_angle.");
    std::array<const double, 20> initJointAng = {0, 0, 0.79, -0.79, -1.57, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.26};  // init joints ang [rad] 
    std::array<const double, 20> initJointVel = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5};  // init joints vel [rad/s]
    for(int i = 0; i < 20; i++) {  // set init position & value
      getJointAng_[i] = 0;
      wb_motor_set_position(motorsTag_[i], initJointAng[i]);
      wb_motor_set_velocity(motorsTag_[i], initJointVel[i]);
    }

    jointNum_legR_ = {6, 8, 10, 12, 14, 16};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(right leg)
    jointNum_legL_ = {7, 9, 11, 13, 15, 17};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(left leg)
    jointAng_posi_or_nega_legR_ = {-1, -1, -1, 1, 1, 1};  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
    jointAng_posi_or_nega_legL_ = {-1, -1, 1, -1, -1, 1}; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)

    RCLCPP_INFO(node_->get_logger(), "Finish init, Start step.");
  }


  void WebotsRobotHandler::callback_res(
    rclcpp::Client<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedFuture future
  ) {
    // callback function

    // std::cout << "RESPONSE"
    //           << "\n" << "__Q_fix_R: ";
    // std::copy(std::begin(future.get()->q_fix_r),
    //           std::end(future.get()->q_fix_r),
    //           std::ostream_iterator<double>(std::cout, " "));
    // std::cout << "\n" << "__Q_fix_L: ";
    // std::copy(std::begin(future.get()->q_fix_l),
    //           std::end(future.get()->q_fix_l),
    //           std::ostream_iterator<double>(std::cout, " "));
    // std::cout << "\n" << "__dQ_fix_R: ";
    // std::copy(std::begin(future.get()->dq_fix_r),
    //           std::end(future.get()->dq_fix_r),
    //           std::ostream_iterator<double>(std::cout, " "));
    // std::cout << "\n" << "__dQ_fix_L: ";
    // std::copy(std::begin(future.get()->dq_fix_l),
    //           std::end(future.get()->dq_fix_l),
    //           std::ostream_iterator<double>(std::cout, " "));
    // std::cout << "\n" << std::endl;

    // set joints angle & velocity
    for(int i = 0; i < 6; i++) {
      wb_motor_set_position(motorsTag_[jointNum_legR_[i]], future.get()->q_fix_r[i]*jointAng_posi_or_nega_legR_[i]);
      wb_motor_set_velocity(motorsTag_[jointNum_legR_[i]], future.get()->dq_fix_r[i]);
      wb_motor_set_position(motorsTag_[jointNum_legL_[i]], future.get()->q_fix_l[i]*jointAng_posi_or_nega_legL_[i]);
      wb_motor_set_velocity(motorsTag_[jointNum_legL_[i]], future.get()->dq_fix_l[i]);
    }
  }


  void WebotsRobotHandler::step() {
    RCLCPP_INFO(node_->get_logger(), "step...");
    
    // DEBUG
    // [DEBUG] wait until the inital movement is over.
    if(count < 300) { std::cout << count << std::endl; count++; }
    else if(count >= 300) {


    // get sensor data
    for(int i = 0; i < 20; i++) {
      getJointAng_[i] = wb_position_sensor_get_value(positionSensorsTag_[i]);
    }
    accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
    gyroValue_ = wb_gyro_get_values(gyroTag_);
  
    auto toWRH_req = std::make_shared<msgs_package::srv::ToWebotsRobotHandlerMessage::Request>();

    // set request (WalkignStabilizationController)
    toWRH_req->accelerometer_now = {accelerometerValue_[0], accelerometerValue_[1], accelerometerValue_[2]};
    toWRH_req->gyro_now = {gyroValue_[0], gyroValue_[1], gyroValue_[2]};
    toWRH_req->q_now_r = {getJointAng_[6], getJointAng_[8], getJointAng_[10], getJointAng_[12], getJointAng_[14], getJointAng_[16]};
    toWRH_req->q_now_l = {getJointAng_[7], getJointAng_[9], getJointAng_[11], getJointAng_[13], getJointAng_[15], getJointAng_[17]};

    // request service (WalkingStabilizationController)
    toWRH_clnt_->async_send_request(
      toWRH_req, 
      std::bind(&WebotsRobotHandler::callback_res, this, _1)
    );

  // DEBUG
    }

  }
}



PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)