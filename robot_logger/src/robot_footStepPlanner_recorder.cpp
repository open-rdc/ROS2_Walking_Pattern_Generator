#include <iostream>
#include <regex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
#include "msgs_package/msg/foot_step_record.hpp"

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

  class RobotFootStepRecorder : public rclcpp::Node {
    public:
      RobotFootStepRecorder(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotFootStepRecorder", options) {
        // auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

        // ファイルの作成。ファイル名先頭に日付時間を付与
        auto time_now = std::chrono::system_clock::now();
        std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        std::string datetime_str = std::ctime(&datetime);
        file_footStep_path = std::regex_replace(datetime_str, std::regex(" "), "_") + "__foot_step.dat";

        file_footStep.open(file_footStep_path, std::ios::out);

        using namespace std::placeholders;
        sub_footStep_ = this->create_subscription<msgs_package::msg::FootStepRecord>("foot_step", 10, std::bind(&RobotFootStepRecorder::FootStep_Callback, this, _1));
      }

      ~RobotFootStepRecorder() {
        file_footStep.close();
      }

    private:
      void FootStep_Callback(const msgs_package::msg::FootStepRecord::SharedPtr callback_data) {

        // データ落ちに対処
        // 落ちたデータの箇所は、今の最新と同値で埋める。
        // CHECKME: データ落ちの箇所は記録しておいたほうが良い？
        diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          for(int loss_step = 1; loss_step < diff; loss_step++) {
            // TODO: datファイルへの書き込みは最後に一括して行いたい。step_count data　って感じで。
            footStep_step_count.push_back(-999);  // loss dataなので、エラー値。いや、単にカウント値を入れるのとエラー値は別にしたほうが良いか？Plotする時を考えると。
            footStep.push_back(footStep.back());
            file_footStep << counter_old_+loss_step << " ";
            for(double acce : callback_data->accelerometer_now) {
              file_footStep << acce << " " << std::endl;
            }
          }
        }
        // record
        footStep_step_count.push_back(callback_data->step_count);
        footStep.push_back(callback_data->accelerometer_now);
        file_footStep << callback_data->step_count << " ";
        for(double acce : callback_data->accelerometer_now) {
          file_footStep << acce << " " << std::endl;
        }

        counter_old_ = callback_data->step_count;
        
      }

      rclcpp::Subscription<msgs_package::msg::footStepRecord>::SharedPtr sub_footStep_;

      // TODO: ファイル名を生成する。../data/内に記録するようにする（../表記が行けるか？無理ならこのフルパスをゲットして記録するか？）
      std::ofstream file_footStep;
      std::string file_footStep_path;

      int loss_count_ = 0;
      int counter_old_ = 0;
      int diff = 0;

      std::vector<std::array<double, 3>> footStep;
      std::vector<int> footStep_step_count;
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Recorder::RobotFootStepRecorder>());
  rclcpp::shutdown();

  return 0;
}