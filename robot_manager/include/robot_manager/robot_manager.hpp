#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/control_output.hpp"
#include "msgs_package/msg/walking_pattern.hpp"
#include "msgs_package/msg/feedback.hpp"
#include "msgs_package/srv/stabilization_control.hpp"
#include "robot_manager/visibility_control.h"

namespace robot_manager {
  class RobotManager : public rclcpp::Node {
    public:
      ROBOT_MANAGER_PUBLIC
      RobotManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      void ControlOutput_Timer();
      void WalkingPattern_Callback(const msgs_package::msg::WalkingPattern::SharedPtr callback_data);
      void Feedback_Callback(const msgs_package::msg::Feedback::SharedPtr callback_data);

      rclcpp::Publisher<msgs_package::msg::ControlOutput>::SharedPtr pub_control_output_;
      rclcpp::Subscription<msgs_package::msg::WalkingPattern>::SharedPtr sub_walking_pattern_;
      rclcpp::Subscription<msgs_package::msg::Feedback>::SharedPtr sub_feedback_;
      rclcpp::Client<msgs_package::srv::StabilizationControl>::SharedPtr clnt_stabilization_control_;
      rclcpp::TimerBase::SharedPtr timer_; 

      rclcpp::CallbackGroup::SharedPtr cb_group1_ = nullptr;
      rclcpp::CallbackGroup::SharedPtr cb_group2_ = nullptr;
  };
}

/* Reference
  https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

*/