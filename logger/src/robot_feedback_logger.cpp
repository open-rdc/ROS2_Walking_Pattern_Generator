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

  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<logger::RobotFeedbackLogger>());
  rclcpp::shutdown();

  return 0;
}