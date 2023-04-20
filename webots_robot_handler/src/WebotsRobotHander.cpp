#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
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
  // auto time = rclcpp::Clock{}.now().seconds();
  // auto time_max = time - time;
  // auto time_min = time + time;
  // int hoge = 0;

  static const rmw_qos_profile_t custom_qos_profile =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
    1,  // History(keep_last) Depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
    RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
    RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
    RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
    false  // avoid_ros_namespace_conventions
  };

  void WebotsRobotHandler::DEBUG_ParameterSetting() {
    motors_name_ = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"), 
                    ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"), 
                    ("Neck"), ("Head")};
    initJointAng_ = {0, 0, -0.5, 0.5, -1, 1, 
                     0, 0, 0, 0, -3.14/8, 3.14/8, 3.14/4, -3.14/4, 3.14/8, -3.14/8, 0, 0, 
                     0, 0.26};  // init joints ang [rad]. corresponding to motors_name
    initJointVel_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5};  // init joints vel [rad/s]

    jointNum_legR_ = {6, 8, 10, 12, 14, 16};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(right leg)
    jointNum_legL_ = {7, 9, 11, 13, 15, 17};  // joint numbers (motorsTag[20] & positionSensorsTag[20])(left leg)
    jointAng_posi_or_nega_legR_ = {-1, -1, 1, 1, -1, 1};  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
    jointAng_posi_or_nega_legL_ = {-1, -1, -1, -1, 1, 1}; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
  }

  void WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    node_ = node;
    auto a = parameters;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start up WebotsRobotHandler. Hello WebotsRobotHandler!!");

    toWRH_clnt_ = node_->create_client<msgs_package::srv::ToWebotsRobotHandlerMessage>(
      "FB_StabilizationController",
      custom_qos_profile
    );

    // check service server
    while(!toWRH_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "ERROR!!: FB_StabilizationController service is dead.");
        return;
      }
      // RCLCPP_INFO(node_->get_logger(), "Waiting for FB_StabilizationController service...");
    }

    // DEBUG parameter setting
    DEBUG_ParameterSetting();

    for(int i = 0; i < 20; i++) {  // get motor tags & position_sensor tags
      motorsTag_[i] = wb_robot_get_device(motors_name_[i].c_str());
      positionSensorsTag_[i] = wb_robot_get_device((motors_name_[i]+"S").c_str());
      wb_position_sensor_enable(positionSensorsTag_[i], 100);
    }
    accelerometerTag_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometerTag_, 100);  // enable & sampling_period: 100[ms]
    gyroTag_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyroTag_, 100);

    // RCLCPP_INFO(node_->get_logger(), "Set init joints_angle.");

    for(int i = 0; i < 20; i++) {  // set init position & value
      getJointAng_[i] = 0;
      wb_motor_set_position(motorsTag_[i], initJointAng_[i]);
      wb_motor_set_velocity(motorsTag_[i], initJointVel_[i]);
    }

    // RCLCPP_INFO(node_->get_logger(), "Finish init, Start step.");
  }


  void WebotsRobotHandler::callback_res(
    rclcpp::Client<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedFuture future
  ) {
    // set joints angle & velocity
    for(int i = 0; i < 6; i++) {
      wb_motor_set_position(motorsTag_[jointNum_legR_[i]], future.get()->q_fix_r[i]*jointAng_posi_or_nega_legR_[i]);
      wb_motor_set_velocity(motorsTag_[jointNum_legR_[i]], future.get()->dq_fix_r[i]);
      wb_motor_set_position(motorsTag_[jointNum_legL_[i]], future.get()->q_fix_l[i]*jointAng_posi_or_nega_legL_[i]);
      wb_motor_set_velocity(motorsTag_[jointNum_legL_[i]], future.get()->dq_fix_l[i]);
    }

    // RCLCPP_INFO(node_->get_logger(), time - rclcpp::Clock{}.now().seconds());
    // auto time2 = rclcpp::Clock{}.now().seconds();
    // if(hoge > 20){
    //   auto time_dev = time2 - time;
    //   if(time_max < time_dev){time_max = time_dev;}
    //   if(time_min > time_dev){time_min = time_dev;}
    //   std::cout << "[WebotsRobotHandler]: " << time_dev << "    max: " << time_max <<  "    min: " << time_min << std::endl;
    // }
    // hoge++;
    // time = time2;
    // RCLCPP_INFO(node_->get_logger(), "Set Robot Motion");
  }


  void WebotsRobotHandler::step() {
    // RCLCPP_INFO(node_->get_logger(), "step...");
    
    // [DEBUG] wait until the inital movement is over.
    if(count < 200) { /*std::cout << count << std::endl;*/ count++; }
    else if(count >= 200) {


      // get sensor data
      for(int i = 0; i < 20; i++) {
        getJointAng_[i] = wb_position_sensor_get_value(positionSensorsTag_[i]);
      }
      accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
      gyroValue_ = wb_gyro_get_values(gyroTag_);
    
      auto toWRH_req = std::make_shared<msgs_package::srv::ToWebotsRobotHandlerMessage::Request>();

      toWRH_req->accelerometer_now = {accelerometerValue_[0], accelerometerValue_[1], accelerometerValue_[2]};
      toWRH_req->gyro_now = {gyroValue_[0], gyroValue_[1], gyroValue_[2]};
      toWRH_req->q_now_r = {getJointAng_[6], getJointAng_[8], getJointAng_[10], getJointAng_[12], getJointAng_[14], getJointAng_[16]};
      toWRH_req->q_now_l = {getJointAng_[7], getJointAng_[9], getJointAng_[11], getJointAng_[13], getJointAng_[15], getJointAng_[17]};

      // RCLCPP_INFO(node_->get_logger(), "Request to WSC...");
      toWRH_clnt_->async_send_request(
        toWRH_req, 
        std::bind(&WebotsRobotHandler::callback_res, this, _1)
      );

    }

    wb_robot_step(1);  // stepの動作周期を固定するために、掛かった処理時間 + a = WPGと同じ制御周期 となるようにしたい。

  }
}


PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)