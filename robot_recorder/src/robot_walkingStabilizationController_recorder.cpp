#include <iostream>
#include <regex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
#include "msgs_package/msg/walking_stabilization_record.hpp"

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

  class RobotWalkingStabilizationRecorder : public rclcpp::Node {
    public:
      RobotWalkingStabilizationRecorder(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotWalkingStabilizationRecorder", options) {
        // auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

        // ファイルの作成。ファイル名先頭に日付時間を付与
        auto time_now = std::chrono::system_clock::now();
        std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        std::string datetime_str = std::ctime(&datetime);
        file_walkingStabilization_path = std::regex_replace(datetime_str, std::regex(" "), "_") + "__walking_stabilization.dat";

        file_walkingStabilization.open(file_walkingStabilization_path, std::ios::out);

        using namespace std::placeholders;
        sub_walkingStabilization_ = this->create_subscription<msgs_package::msg::WalkingStabilizationRecord>("walking_stabilization", 10, std::bind(&RobotWalkingStabilizationRecorder::WalkingStabilization_Callback, this, _1));
      }

      ~RobotWalkingStabilizationRecorder() {
        file_walkingStabilization.close();
      }

    private:
      void WalkingStabilization_Callback(const msgs_package::msg::WalkingStabilizationRecord::SharedPtr callback_data) {

        // データ落ちに対処
        // 落ちたデータの箇所は、今の最新と同値で埋める。
        // CHECKME: データ落ちの箇所は記録しておいたほうが良い？
        diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          for(int loss_step = 1; loss_step < diff; loss_step++) {
            // TODO: datファイルへの書き込みは最後に一括して行いたい。step_count data　って感じで。
            walkingStabilization_step_count.push_back(-999);  // loss dataなので、エラー値。いや、単にカウント値を入れるのとエラー値は別にしたほうが良いか？Plotする時を考えると。
            walkingStabilization.push_back(walkingStabilization.back());
            file_walkingStabilization << counter_old_+loss_step << " ";
            for(double acce : callback_data->accelerometer_now) {
              file_walkingStabilization << acce << " " << std::endl;
            }
          }
        }
        // record
        walkingStabilization_step_count.push_back(callback_data->step_count);
        walkingStabilization.push_back(callback_data->accelerometer_now);
        file_walkingStabilization << callback_data->step_count << " ";
        for(double acce : callback_data->accelerometer_now) {
          file_walkingStabilization << acce << " " << std::endl;
        }

        counter_old_ = callback_data->step_count;
        
      }

      rclcpp::Subscription<msgs_package::msg::WalkingStabilizationRecord>::SharedPtr sub_walkingStabilization_;

      // TODO: ファイル名を生成する。../data/内に記録するようにする（../表記が行けるか？無理ならこのフルパスをゲットして記録するか？）
      std::ofstream file_walkingStabilization;
      std::string file_walkingStabilization_path;

      int loss_count_ = 0;
      int counter_old_ = 0;
      int diff = 0;

      std::vector<std::array<double, 3>> walkingStabilization;
      std::vector<int> walkingStabilization_step_count;
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Recorder::RobotWalkingStabilizationRecorder>());
  rclcpp::shutdown();

  return 0;
}