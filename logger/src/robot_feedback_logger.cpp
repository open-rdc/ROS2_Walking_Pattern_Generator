#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "msgs_package/msg/feedback.hpp"

#include <fstream>

namespace logger {
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

  class RobotFeedbackLogger : public rclcpp::Node {
    public:
      RobotFeedbackLogger(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotFeedbackLogger", options) {
        auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

        WRH_log_Feedback.open(WRH_log_Feedback_path, std::ios::out);

        using namespace std::placeholders;
        sub_feedback_ = this->create_subscription<msgs_package::msg::Feedback>("Feedback", custom_QoS, std::bind(&RobotFeedbackLogger::Feedback_Callback, this, _1));
      }

      ~RobotFeedbackLogger() {
        WRH_log_Feedback.close();
      }

    private:
      void Feedback_Callback(const msgs_package::msg::Feedback::SharedPtr callback_data) {
/* Accelerometer & Gyro. Darwin-op.proto 仕様 (Pub側で対処済み)
source: https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto

        Accelerometer {
          translation -0.01 0 -0.068
          rotation 0 0 1.0 -1.5708  # z軸基準に座標を-90°回転 (x -> -y, y -> x, z -> z)
          name "Accelerometer"
          lookupTable [
            -39.24 0 0 39.24 1024 0
          ]
        }
        Gyro {
          translation 0.01 0 -0.068
          rotation 0 0 1.0 -3.1416  # z軸基準に座標を-180°回転 (x -> -x, y -> -y, z -> z)
          name "Gyro"
          lookupTable [
            -27.925 0 0 27.925 1024 0
          ]
        }

      Offset (センサデータ出力値より推測)
      Acce
        x: 512
        y: 512
        z: 640
      Gyro
        x: 512
        y: 512
        z: 512
        
reference:
  acce: https://github.com/cyberbotics/webots/blob/master/docs/reference/accelerometer.md
  gyro: https://github.com/cyberbotics/webots/blob/master/docs/reference/gyro.md
*/
        // データ落ちに対処
        // 落ちたデータの箇所は、今の最新と同値で埋める。
        // CHECKME: データ落ちの箇所は記録しておいたほうが良い？
        int diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          loss_count_++;
          if(2 == diff) {
            // RCLCPP_WARN(this->get_logger(), "Feedback Data Loss!!: loss count [ %d ], loss data step number [ %d ]", loss_count_, counter_old_+1);
            WRH_log_Feedback << counter_old_+1 << " ";
            for(double acce : callback_data->accelerometer_now) {
              WRH_log_Feedback << acce << " ";
            }
            for(double gyro : callback_data->gyro_now) {
              WRH_log_Feedback << gyro << " ";
            }
            WRH_log_Feedback << std::endl;

          }
          else {
            for(int loss_step = 1; loss_step < diff; loss_step++) {
              // RCLCPP_WARN(this->get_logger(), "Feedback Data Loss!!: loss count [ %d ], loss data step number [ %d ]", loss_count_, counter_old_+loss_step);
              WRH_log_Feedback << counter_old_+loss_step << " ";
              for(double acce : callback_data->accelerometer_now) {
                WRH_log_Feedback << acce << " ";
              }
              for(double gyro : callback_data->gyro_now) {
                WRH_log_Feedback << gyro << " ";
              }
              WRH_log_Feedback << std::endl;
            }
          }
        }
        counter_old_ = callback_data->step_count;
        // accelerometer_old_ = callback_data->accelerometer_now;
        // gyro_old_ = callback_data->gyro_now;

        // ファイルに出力
        // TODO: リアルタイムでグラフにプロットしたい。
        // TODO: 行数で、ファイルを複数に分けたい。１つのファイルに出力し続けたら行数がやばいことになる。
        WRH_log_Feedback << callback_data->step_count << " ";
        for(double acce : callback_data->accelerometer_now) {
          WRH_log_Feedback << acce << " ";
        }
        for(double gyro : callback_data->gyro_now) {
          WRH_log_Feedback << gyro << " ";
        }
        WRH_log_Feedback << std::endl;
      }

      rclcpp::Subscription<msgs_package::msg::Feedback>::SharedPtr sub_feedback_;

      std::ofstream WRH_log_Feedback;
      std::string WRH_log_Feedback_path = "src/Log/WRH_log_Feedback.dat";

      int loss_count_ = 0;
      int counter_old_ = 0;

      // std::array<double, 3> accelerometer_old_ = {0, 0, 0};
      // std::array<double, 3> gyro_old_ = {0, 0, 0};

  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<logger::RobotFeedbackLogger>());
  rclcpp::shutdown();

  return 0;
}