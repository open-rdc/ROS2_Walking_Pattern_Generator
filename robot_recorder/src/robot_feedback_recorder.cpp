#include <iostream>
#include <regex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
#include "robot_messages/msg/feedback.hpp"

#include <fstream>

namespace Recorder {
  // static const rmw_qos_profile_t custom_qos_profile =
  // {
  //   RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
  //   1,  // History(keep_last) Depth
  //   RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
  //   RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
  //   RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
  //   RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
  //   RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
  //   RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
  //   false  // avoid_ros_namespace_conventions
  // };

  class RobotFeedbackRecorder : public rclcpp::Node {
    public:
      RobotFeedbackRecorder(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotFeedbackRecorder", options) {
        // auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

        // ファイルの作成。ファイル名先頭に日付時間を付与
        auto time_now = std::chrono::system_clock::now();
        std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        std::string datetime_str = std::ctime(&datetime);
        std::cout << datetime_str << std::endl;
        file_feedback_acc_path = std::regex_replace(datetime_str, std::regex(" "), "_") + "__feedback-acceleration.dat";
        file_feedback_gyro_path = std::regex_replace(datetime_str, std::regex(" "), "_") + "__feedback-gyro.dat";

        file_feedback_acc.open(file_feedback_acc_path, std::ios::out);
        file_feedback_gyro.open(file_feedback_gyro_path, std::ios::out);

        std::cout << "hoge" << std::endl;
        file_feedback_acc << "hoge" << std::endl;

        using namespace std::placeholders;
        sub_feedback_ = this->create_subscription<robot_messages::msg::Feedback>("Feedback", 10, std::bind(&RobotFeedbackRecorder::Feedback_Callback, this, _1));
      }

      ~RobotFeedbackRecorder() {
        file_feedback_acc.close();
        file_feedback_gyro.close();
        std::cout << "end" << std::endl;
      }

    private:
      void Feedback_Callback(const robot_messages::msg::Feedback::SharedPtr callback_data) {
/* Accelerometer & Gyro. Darwin-op.proto 仕様
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
        diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          // RCLCPP_WARN(this->get_Recorder(), "Feedback Data Loss!!: loss count [ %d ], loss data step number [ %d ]", loss_count_, counter_old_+1);
          for(int loss_step = 1; loss_step < diff; loss_step++) {
            // TODO: datファイルへの書き込みは最後に一括して行いたい。step_count data　って感じで。
            feedback_step_count.push_back(-999);  // loss dataなので、エラー値。いや、単にカウント値を入れるのとエラー値は別にしたほうが良いか？Plotする時を考えると。
            feedback_acceleration.push_back(feedback_acceleration.back());
            feedback_gyro.push_back(feedback_gyro.back());
            file_feedback_acc << counter_old_+loss_step << " ";
            file_feedback_gyro << counter_old_+loss_step << " ";

            for(double acce : feedback_acceleration.back()) {
              file_feedback_acc << acce << " " << std::endl;
            }
            for(double gyro : feedback_gyro.back()) {
              file_feedback_gyro << gyro << " " << std::endl;
            }
          }
        }
        // record
        feedback_step_count.push_back(callback_data->step_count);
        feedback_acceleration.push_back(callback_data->accelerometer_now);
        feedback_gyro.push_back(callback_data->gyro_now);
        file_feedback_acc << callback_data->step_count << " ";
        file_feedback_gyro << callback_data->step_count << " ";

        for(double acce : callback_data->accelerometer_now) {
          file_feedback_acc << acce << " " << std::endl;
        }
        for(double gyro : callback_data->gyro_now) {
          file_feedback_gyro << gyro << " " << std::endl;
        }

        counter_old_ = callback_data->step_count;
        
      }

      rclcpp::Subscription<robot_messages::msg::Feedback>::SharedPtr sub_feedback_;

      // TODO: ファイル名を生成する。../data/内に記録するようにする（../表記が行けるか？無理ならこのフルパスをゲットして記録するか？）
      std::ofstream file_feedback_acc;
      std::ofstream file_feedback_gyro;
      std::string file_feedback_acc_path;
      std::string file_feedback_gyro_path;

      int loss_count_ = 0;
      int counter_old_ = 0;
      int diff = 0;

      std::vector<std::array<double, 3>> feedback_acceleration;
      std::vector<std::array<double, 3>> feedback_gyro;
      std::vector<int> feedback_step_count;
  };
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Recorder::RobotFeedbackRecorder>());
  rclcpp::shutdown();

  return 0;
}