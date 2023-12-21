#include <iostream>
// #include <regex>
// #include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
#include "robot_messages/msg/walking_pattern_record.hpp"

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

  class RobotWalkingPatternRecorder : public rclcpp::Node {
    public:
      RobotWalkingPatternRecorder(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotWalkingPatternRecorder", options) {
        // auto custom_QoS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile));

        // ファイルの作成。ファイル名先頭に日付時間を付与
        // auto time_now = std::chrono::system_clock::now();
        // std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        // std::string datetime_str = std::ctime(&datetime);

        std::string record_dir_path = get_parameter("record_dir_path").as_string();
        std::string launch_datetime = get_parameter("launch_datetime").as_string();

        // file_walkingPattern_pos_path = record_dir_path + launch_datetime + "__walking_pattern-position.dat";
        // file_walkingPattern_vel_path = record_dir_path + launch_datetime + "__walking_pattern-velocity.dat";
        file_walkingPattern_path = record_dir_path + launch_datetime + "__walking_pattern.dat";

        // file_walkingPattern_pos.open(file_walkingPattern_pos_path, std::ios::out);
        // file_walkingPattern_vel.open(file_walkingPattern_pos_path, std::ios::out);
        file_walkingPattern.open(file_walkingPattern_path, std::ios::out);

        file_walkingPattern << "# record data: step_count | walking_pattern_position (x y z) | walking_pattern_velocity (x y z) | walking_pattern_zmp (x y)" << std::endl;

        using namespace std::placeholders;
        sub_walkingPattern_ = this->create_subscription<robot_messages::msg::WalkingPatternRecord>("walking_pattern", 10, std::bind(&RobotWalkingPatternRecorder::WalkingPattern_Callback, this, _1));
      }

      ~RobotWalkingPatternRecorder() {
        // file_walkingPattern_pos.close();
        // file_walkingPattern_vel.close();
        file_walkingPattern.close();
      }

    private:
      void WalkingPattern_Callback(const robot_messages::msg::WalkingPatternRecord::SharedPtr callback_data) {

        // データ落ちに対処
        // 落ちたデータの箇所は、今の最新と同値で埋める。
        // CHECKME: データ落ちの箇所は記録しておいたほうが良い？
        diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          for(int loss_step = 1; loss_step < diff; loss_step++) {
            // TODO: datファイルへの書き込みは最後に一括して行いたい。step_count data　って感じで。
            walkingPattern_step_count.push_back(-999);  // loss dataなので、エラー値。いや、単にカウント値を入れるのとエラー値は別にしたほうが良いか？Plotする時を考えると。
            walkingPattern_pos.push_back(walkingPattern_pos.back());
            walkingPattern_vel.push_back(walkingPattern_vel.back());
            walkingPattern_zmp_pos.push_back(walkingPattern_zmp_pos.back());

            // file_walkingPattern_pos << counter_old_+loss_step << " ";
            // file_walkingPattern_vel << counter_old_+loss_step << " ";
            file_walkingPattern << counter_old_+loss_step << " ";
            for(double pos : walkingPattern_pos.back()) {
              file_walkingPattern << pos << " ";
            }
            for(double vel : walkingPattern_vel.back()) {
              file_walkingPattern << vel << " ";
            }
            for(double zmp : walkingPattern_zmp_pos.back()) {
              file_walkingPattern << zmp << " ";
            }
            file_walkingPattern << std::endl;
          }
        }
        // record
        walkingPattern_step_count.push_back(callback_data->step_count);
        walkingPattern_pos.push_back(callback_data->cc_cog_pos_ref);
        walkingPattern_vel.push_back(callback_data->cc_cog_vel_ref);
        walkingPattern_zmp_pos.push_back(callback_data->wc_foot_land_pos_ref);

        // file_walkingPattern_pos << callback_data->step_count << " ";
        // file_walkingPattern_vel << callback_data->step_count << " ";
        file_walkingPattern << callback_data->step_count << " ";
        for(double pos : callback_data->cc_cog_pos_ref) {
          file_walkingPattern << pos << " ";
        }
        for(double vel : callback_data->cc_cog_vel_ref) {
          file_walkingPattern << vel << " ";
        }
        for(double zmp : callback_data->wc_foot_land_pos_ref) {
          file_walkingPattern << zmp << " ";
        }
        file_walkingPattern << std::endl;

        counter_old_ = callback_data->step_count;
        
      }

      rclcpp::Subscription<robot_messages::msg::WalkingPatternRecord>::SharedPtr sub_walkingPattern_;

      // std::ofstream file_walkingPattern_pos;
      // std::ofstream file_walkingPattern_vel;
      // std::string file_walkingPattern_pos_path;
      // std::string file_walkingPattern_vel_path;
      std::ofstream file_walkingPattern;
      std::string file_walkingPattern_path;

      int loss_count_ = 0;
      int counter_old_ = 0;
      int diff = 0;

      std::vector<std::array<double, 3>> walkingPattern_pos;
      std::vector<std::array<double, 3>> walkingPattern_vel;
      std::vector<std::array<double, 2>> walkingPattern_zmp_pos;
      std::vector<int> walkingPattern_step_count;
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_option;
  node_option.allow_undeclared_parameters(true);
  node_option.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<Recorder::RobotWalkingPatternRecorder>(node_option));
  rclcpp::shutdown();

  return 0;
}