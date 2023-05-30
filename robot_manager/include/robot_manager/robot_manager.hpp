#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/control_output.hpp"
#include "msgs_package/msg/walking_pattern.hpp"
#include "msgs_package/msg/feedback.hpp"
#include "msgs_package/srv/stabilization_control.hpp"

namespace robot_manager {
  class RobotManager : public rclcpp::Node {
    public:
      RobotManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      void ControlOutput_Timer();
      void WalkingPattern_Callback(const msgs_package::msg::WalkingPattern::SharedPtr callback_data);
      void Feedback_Callback(const msgs_package::msg::Feedback::SharedPtr callback_data);

      rclcpp::Publisher<msgs_package::msg::ControlOutput>::SharedPtr pub_control_output_;
      rclcpp::TimerBase::SharedPtr timer_; 

      rclcpp::Client<msgs_package::srv::StabilizationControl>::SharedPtr clnt_stabilization_control_;
      rclcpp::Subscription<msgs_package::msg::WalkingPattern>::SharedPtr sub_walking_pattern_;
      rclcpp::Subscription<msgs_package::msg::Feedback>::SharedPtr sub_feedback_;

      rclcpp::CallbackGroup::SharedPtr cb_group_ = nullptr;
      rclcpp::CallbackGroup::SharedPtr cc_group_ = nullptr;
  };
}