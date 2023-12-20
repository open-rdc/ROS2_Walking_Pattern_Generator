#include <iostream>
// #include <regex>
// #include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include <rmw/qos_profiles.h>
// #include "robot_messages/msg/JointStates.hpp"
#include "robot_messages/msg/joint_state_record.hpp"

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
          // -TODO: Launch時にYear-Month-Day-Hour-Minuteの文字列をParameterとして受け取って、それをファイル名にしよう。
            // -TODO: そしてファイルも多くなるから、その文字列が名前のディレクトリをLaunch時に作ってもらおう。Debug ModeがONならね。
        std::string record_dir_path = get_parameter("record_dir_path").as_string();
        std::string launch_datetime = get_parameter("launch_datetime").as_string();
        // auto time_now = std::chrono::system_clock::now();
        // std::time_t datetime = std::chrono::system_clock::to_time_t(time_now);
        // std::string datetime_str = std::ctime(&datetime);

        // file_jointStates_ang_legL_path = record_dir_path + launch_datetime + "__joint_states-legL-position.dat";
        // file_jointStates_ang_legR_path = record_dir_path + launch_datetime + "__joint_states-legR-position.dat";
        // file_jointStates_vel_legL_path = record_dir_path + launch_datetime + "__joint_states-legL-veloctiy.dat";
        // file_jointStates_vel_legR_path = record_dir_path + launch_datetime + "__joint_states-legR-velocity.dat";
        file_jointStates_path = record_dir_path + launch_datetime + "__joint_states.dat";

        // file_jointStates_ang_legL.open(file_jointStates_ang_legL_path, std::ios::out);
        // file_jointStates_ang_legR.open(file_jointStates_ang_legR_path, std::ios::out);
        // file_jointStates_vel_legL.open(file_jointStates_vel_legL_path, std::ios::out);
        // file_jointStates_ang_legR.open(file_jointStates_vel_legR_path, std::ios::out);
        file_jointStates.open(file_jointStates_path, std::ios::out);

        file_jointStates << "# record data: step_count | legL_position (6-joints [rad] value) |  legR_position (6-joints [rad] value) |  legL_velocity (6-joints [rad/s] value) |  legR_velocity (6-joints [rad/s] value)" << std::endl;

        using namespace std::placeholders;
        sub_jointStates_ = this->create_subscription<robot_messages::msg::JointStateRecord>("joint_states_record", 10, std::bind(&RobotJointStatesRecorder::JointStates_Callback, this, _1));
      }

      ~RobotJointStatesRecorder() {
        // file_jointStates_ang_legL.close();
        // file_jointStates_ang_legR.close();
        // file_jointStates_vel_legL.close();
        // file_jointStates_vel_legR.close();
      file_jointStates.close();
      }

    private:
      void JointStates_Callback(const robot_messages::msg::JointStateRecord::SharedPtr callback_data) {
        // データ落ちに対処
        // 落ちたデータの箇所は、今の最新と同値で埋める。
        // CHECKME: データ落ちの箇所は記録しておいたほうが良い？
        diff = callback_data->step_count - counter_old_;
        if(1 != diff) {
          for(int loss_step = 1; loss_step < diff; loss_step++) {
            // TODO: datファイルへの書き込みは最後に一括して行いたい。step_count data　って感じで。
            jointStates_step_count.push_back(-999);  // loss dataなので、エラー値。いや、単にカウント値を入れるのとエラー値は別にしたほうが良いか？Plotする時を考えると。
            jointStates_ang_legL.push_back(jointStates_ang_legL.back());
            jointStates_ang_legR.push_back(jointStates_ang_legR.back());
            jointStates_vel_legL.push_back(jointStates_vel_legL.back());
            jointStates_vel_legR.push_back(jointStates_vel_legR.back());

            // file_jointStates_ang_legL << counter_old_+loss_step << " ";
            // file_jointStates_ang_legR << counter_old_+loss_step << " ";
            // file_jointStates_vel_legL << counter_old_+loss_step << " ";
            // file_jointStates_vel_legR << counter_old_+loss_step << " ";
            file_jointStates << counter_old_+loss_step << " ";
            for(double angL : jointStates_ang_legL.back()) {
              file_jointStates << angL << " ";
            }
            for(double angR : jointStates_ang_legR.back()) {
              file_jointStates << angR << " ";
            }
            for(double velL : jointStates_vel_legL.back()) {
              file_jointStates << velL << " ";
            }
            for(double velR : jointStates_vel_legR.back()) {
              file_jointStates << velR << " ";
            }
            file_jointStates << std::endl;
          }
        }
        // record
        jointStates_step_count.push_back(callback_data->step_count);
        jointStates_ang_legL.push_back(callback_data->joint_ang_leg_l);
        jointStates_ang_legR.push_back(callback_data->joint_ang_leg_r);
        jointStates_vel_legL.push_back(callback_data->joint_vel_leg_l);
        jointStates_vel_legR.push_back(callback_data->joint_vel_leg_r);

        // file_jointStates_ang_legL << callback_data->step_count << " ";
        // file_jointStates_ang_legR << callback_data->step_count << " ";
        // file_jointStates_vel_legL << callback_data->step_count << " ";
        // file_jointStates_vel_legR << callback_data->step_count << " ";
        file_jointStates << callback_data->step_count << " ";
        for(double angL : callback_data->joint_ang_leg_l) {
          file_jointStates << angL << " ";
        }
        for(double angR : callback_data->joint_ang_leg_r) {
          file_jointStates << angR << " ";
        }
        for(double velL : callback_data->joint_vel_leg_l) {
          file_jointStates << velL << " ";
        }
        for(double velR : callback_data->joint_vel_leg_r) {
          file_jointStates << velR << " ";
        }
        file_jointStates << std::endl;

        counter_old_ = callback_data->step_count;
        
      }

      rclcpp::Subscription<robot_messages::msg::JointStateRecord>::SharedPtr sub_jointStates_;

      // std::ofstream file_jointStates_ang_legL;
      // std::ofstream file_jointStates_ang_legR;
      // std::ofstream file_jointStates_vel_legL;
      // std::ofstream file_jointStates_vel_legR;
      // std::string file_jointStates_ang_legL_path;
      // std::string file_jointStates_ang_legR_path;
      // std::string file_jointStates_vel_legL_path;
      // std::string file_jointStates_vel_legR_path;
      std::ofstream file_jointStates;
      std::string file_jointStates_path;

      int loss_count_ = 0;
      int counter_old_ = 0;
      int diff = 0;

      std::vector<std::array<double, 6>> jointStates_ang_legL;
      std::vector<std::array<double, 6>> jointStates_ang_legR;
      std::vector<std::array<double, 6>> jointStates_vel_legL;
      std::vector<std::array<double, 6>> jointStates_vel_legR;
      std::vector<int> jointStates_step_count;
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_option;
  node_option.allow_undeclared_parameters(true);
  node_option.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<Recorder::RobotJointStatesRecorder>(node_option));
  rclcpp::shutdown();

  return 0;
}