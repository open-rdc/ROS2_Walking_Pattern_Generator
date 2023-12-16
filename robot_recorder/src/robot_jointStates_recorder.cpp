#include <iostream>
#include <regex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
// #include "msgs_package/msg/JointStates.hpp"
#include "msgs_package/msg/joint_state_record.hpp"

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

  class RobotJointStatesRecorder : public rclcpp::Node {
    public:
      RobotJointStatesRecorder(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotJointStatesRecorder", options) {
        // auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

        // ファイルの作成。ファイル名先頭に日付時間を付与
        auto time_now = std::chrono::system_clock::now();
        std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        std::string datetime_str = std::ctime(&datetime);
        file_jointStates_path = std::regex_replace(datetime_str, std::regex(" "), "_") + "__joint_states.dat";

        file_jointStates.open(file_jointStates_path, std::ios::out);

        using namespace std::placeholders;
        sub_jointStates_ = this->create_subscription<msgs_package::msg::JointStateRecord>("joint_states_record", 10, std::bind(&RobotJointStatesRecorder::JointStates_Callback, this, _1));
      }

      ~RobotJointStatesRecorder() {
        file_jointStates.close();
      }

    private:
      void JointStates_Callback(const msgs_package::msg::JointStateRecord::SharedPtr callback_data) {

        // record
        jointStates.push_back(callback_data->accelerometer_now);
        for(double acce : callback_data->accelerometer_now) {
          file_jointStates << acce << " " << std::endl;
        }

      }

      rclcpp::Subscription<msgs_package::msg::JointStateRecord>::SharedPtr sub_jointStates_;

      // TODO: ファイル名を生成する。../data/内に記録するようにする（../表記が行けるか？無理ならこのフルパスをゲットして記録するか？）
      std::ofstream file_jointStates;
      std::string file_jointStates_path;

      std::vector<std::array<double, 3>> jointStates;
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Recorder::RobotJointStatesRecorder>());
  rclcpp::shutdown();

  return 0;
}