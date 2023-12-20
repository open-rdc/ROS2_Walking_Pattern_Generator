#include <iostream>
// #include <regex>
// #include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
#include "robot_messages/msg/walking_stabilization_record.hpp"

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
        // auto time_now = std::chrono::system_clock::now();
        // std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        // std::string datetime_str = std::ctime(&datetime);

        std::string record_dir_path = get_parameter("record_dir_path").as_string();
        std::string launch_datetime = get_parameter("launch_datetime").as_string();

        // file_walkingStabilization_pos_path = record_dir_path + launch_datetime + "__walking_stabilization-cog-position.dat";
        // file_walkingStabilization_vel_path = record_dir_path + launch_datetime + "__walking_stabilization-cog-velocity.dat";
        // file_walkingStabilization_zmp_pos_path = record_dir_path + launch_datetime + "__walking_stabilization-zmp-position.dat";
        file_walkingStabilization_path = record_dir_path + launch_datetime + "__walking_stabilization.dat";

        // file_walkingStabilization_pos.open(file_walkingStabilization_pos_path, std::ios::out);
        // file_walkingStabilization_vel.open(file_walkingStabilization_vel_path, std::ios::out);
        // file_walkingStabilization_zmp_pos.open(file_walkingStabilization_zmp_pos_path, std::ios::out);
        file_walkingStabilization.open(file_walkingStabilization_path, std::ios::out);

        file_walkingStabilization << "# record data: step_count | fixed_CoG_position (x y z) | fixed_CoG_velocity (x y z) | fixed_ZMP_position (x y z)" << std::endl;

        using namespace std::placeholders;
        sub_walkingStabilization_ = this->create_subscription<robot_messages::msg::WalkingStabilizationRecord>("walking_stabilization", 10, std::bind(&RobotWalkingStabilizationRecorder::WalkingStabilization_Callback, this, _1));
      }

      ~RobotWalkingStabilizationRecorder() {
        // file_walkingStabilization_pos.close();
        // file_walkingStabilization_vel.close();
        // file_walkingStabilization_zmp_pos.close();
        file_walkingStabilization.close();
      }

    private:
      void WalkingStabilization_Callback(const robot_messages::msg::WalkingStabilizationRecord::SharedPtr callback_data) {

        // データ落ちに対処
        // 落ちたデータの箇所は、今の最新と同値で埋める。
        // CHECKME: データ落ちの箇所は記録しておいたほうが良い？
        diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          for(int loss_step = 1; loss_step < diff; loss_step++) {
            // TODO: datファイルへの書き込みは最後に一括して行いたい。step_count data　って感じで。
            walkingStabilization_step_count.push_back(-999);  // loss dataなので、エラー値。いや、単にカウント値を入れるのとエラー値は別にしたほうが良いか？Plotする時を考えると。
            walkingStabilization_pos.push_back(walkingStabilization_pos.back());
            walkingStabilization_vel.push_back(walkingStabilization_vel.back());
            walkingStabilization_zmp_pos.push_back(walkingStabilization_zmp_pos.back());

            // file_walkingStabilization_pos << counter_old_+loss_step << " ";
            // file_walkingStabilization_vel << counter_old_+loss_step << " ";
            // file_walkingStabilization_zmp_pos << counter_old_+loss_step << " ";
            file_walkingStabilization << counter_old_+loss_step << " ";
            for(double pos : walkingStabilization_pos.back()) {
              file_walkingStabilization << pos << " ";
            }
            for(double vel : walkingStabilization_vel.back()) {
              file_walkingStabilization << vel << " ";
            }
            for(double zmp : walkingStabilization_zmp_pos.back()) {
              file_walkingStabilization << zmp << " ";
            }
            file_walkingStabilization << std::endl;
          }
        }
        // record
        walkingStabilization_step_count.push_back(callback_data->step_count);
        walkingStabilization_pos.push_back(callback_data->cog_pos_fix);
        walkingStabilization_vel.push_back(callback_data->cog_vel_fix);
        walkingStabilization_zmp_pos.push_back(callback_data->zmp_pos_fix);

        // file_walkingStabilization_pos << callback_data->step_count << " ";
        // file_walkingStabilization_vel << callback_data->step_count << " ";
        // file_walkingStabilization_zmp_pos << callback_data->step_count << " ";
        file_walkingStabilization << callback_data->step_count << " ";
        for(double pos : callback_data->cog_pos_fix) {
          file_walkingStabilization << pos << " ";
        }
        for(double vel : callback_data->cog_vel_fix) {
          file_walkingStabilization << vel << " ";
        }
        for(double zmp : callback_data->zmp_pos_fix) {
          file_walkingStabilization << zmp << " ";
        }
        file_walkingStabilization << std::endl;

        counter_old_ = callback_data->step_count;
        
      }

      rclcpp::Subscription<robot_messages::msg::WalkingStabilizationRecord>::SharedPtr sub_walkingStabilization_;

      // std::ofstream file_walkingStabilization_pos;
      // std::ofstream file_walkingStabilization_vel;
      // std::ofstream file_walkingStabilization_zmp_pos;
      // std::string file_walkingStabilization_pos_path;
      // std::string file_walkingStabilization_vel_path;
      // std::string file_walkingStabilization_zmp_pos_path;
      std::ofstream file_walkingStabilization;
      std::string file_walkingStabilization_path;

      int loss_count_ = 0;
      int counter_old_ = 0;
      int diff = 0;

      std::vector<std::array<double, 3>> walkingStabilization_pos;
      std::vector<std::array<double, 3>> walkingStabilization_vel;
      std::vector<std::array<double, 2>> walkingStabilization_zmp_pos;
      std::vector<int> walkingStabilization_step_count;
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_option;
  node_option.allow_undeclared_parameters(true);
  node_option.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<Recorder::RobotWalkingStabilizationRecorder>(node_option));
  rclcpp::shutdown();

  return 0;
}