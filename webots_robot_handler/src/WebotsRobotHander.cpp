#include "webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "msgs_package/srv/to_robot_manager.hpp"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace webots_robot_handler
{
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
    motors_name_ = {("ShoulderR"), ("ShoulderL"), ("ArmUpperR"), ("ArmUpperL"), ("ArmLowerR"), ("ArmLowerL"), ("PelvYR"), ("PelvYL"), ("PelvR"), ("PelvL"), ("LegUpperR"), ("LegUpperL"), ("LegLowerR"), ("LegLowerL"), ("AnkleR"), ("AnkleL"), ("FootR"), ("FootL"), ("Neck"), ("Head")};
    initJointAng_ = {0, 0, 0.79, -0.79, -1.57, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.26};  // init joints ang [rad] 
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
    auto hoge = parameters;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start up WebotsRobotHandler. Hello WebotsRobotHandler!!");

    RM_clnt_ = node_->create_client<msgs_package::srv::ToRobotManager>(
      "RobotManage",
      custom_qos_profile,
      callback_group_
    );

    // check & wait service server
    while(!RM_clnt_->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "ERROR!!: RobotManage service is dead.");
        return;
      }
      // RCLCPP_INFO(node_->get_logger(), "Waiting for RobotManage service...");
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

    step_count_ = 0;

    // RCLCPP_INFO(node_->get_logger(), "Finish init, Start step.");
  }

  // void WebotsRobotHandler::callback_res(
  //   rclcpp::Client<msgs_package::srv::ToRobotManager>::SharedFuture future
  // ) {
  //   // set joints angle & velocity
  //   // for(int i = 0; i < 6; i++) {
  //   //   wb_motor_set_position(motorsTag_[jointNum_legR_[i]], RM_future.get()->q_next_leg_r[i]*jointAng_posi_or_nega_legR_[i]);
  //   //   wb_motor_set_velocity(motorsTag_[jointNum_legR_[i]], RM_future.get()->dq_next_leg_r[i]);
  //   //   wb_motor_set_position(motorsTag_[jointNum_legL_[i]], RM_future.get()->q_next_leg_l[i]*jointAng_posi_or_nega_legL_[i]);
  //   //   wb_motor_set_velocity(motorsTag_[jointNum_legL_[i]], RM_future.get()->dq_next_leg_l[i]);
  //   // }
  // }

  void WebotsRobotHandler::step() {

    step_count_++;

    // get now status
    for(int i = 0; i < 20; i++) {
      getJointAng_[i] = wb_position_sensor_get_value(positionSensorsTag_[i]);
    }
    accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
    gyroValue_ = wb_gyro_get_values(gyroTag_);

    // set request
    auto RM_clnt_req = std::make_shared<msgs_package::srv::ToRobotManager::Request>();

    RM_clnt_req->q_now_leg_r = {getJointAng_[jointNum_legR_[0]],
                                getJointAng_[jointNum_legR_[1]],
                                getJointAng_[jointNum_legR_[2]],
                                getJointAng_[jointNum_legR_[3]],
                                getJointAng_[jointNum_legR_[4]],
                                getJointAng_[jointNum_legR_[5]],}; 
                                
    RM_clnt_req->q_now_leg_l = {getJointAng_[jointNum_legL_[0]],
                                getJointAng_[jointNum_legL_[1]],
                                getJointAng_[jointNum_legL_[2]],
                                getJointAng_[jointNum_legL_[3]],
                                getJointAng_[jointNum_legL_[4]],
                                getJointAng_[jointNum_legL_[5]],}; 

    RM_clnt_req->accelerometer_now = {accelerometerValue_[0], 
                                      accelerometerValue_[1], 
                                      accelerometerValue_[2]};

    RM_clnt_req->gyro_now = {gyroValue_[0],
                              gyroValue_[1],
                              gyroValue_[2]};

    RM_clnt_req->step_count = step_count_;

    // requset & wait response
    auto RM_future = RM_clnt_->async_send_request(RM_clnt_req);
    // RM_future.wait();
    // std::future_status future_status = RM_future.wait_for(10ms);  // wait for 10ms
    // if(future_status == std::future_status::ready) {
      for(int i = 0; i < 6; i++) {
        wb_motor_set_position(motorsTag_[jointNum_legR_[i]], RM_future.get()->q_next_leg_r[i]*jointAng_posi_or_nega_legR_[i]);
        wb_motor_set_velocity(motorsTag_[jointNum_legR_[i]], RM_future.get()->dq_next_leg_r[i]);
        wb_motor_set_position(motorsTag_[jointNum_legL_[i]], RM_future.get()->q_next_leg_l[i]*jointAng_posi_or_nega_legL_[i]);
        wb_motor_set_velocity(motorsTag_[jointNum_legL_[i]], RM_future.get()->dq_next_leg_l[i]);
      }
    // }
    // else if(future_status == std::future_status::timeout) {
    //   RCLCPP_WARN(node_->get_logger(), "<TIMEOUT> RM_future: Time over 10ms");
    // }
  }
}

PLUGINLIB_EXPORT_CLASS (
  webots_robot_handler::WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)