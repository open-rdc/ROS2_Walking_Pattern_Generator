#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_messages/msg/feedback.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"

#include "Eigen/Dense"

#include <chrono>

namespace robot_manager
{  
  class RobotManager : public rclcpp::Node {
    public:
      RobotManager(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
      );
      ~RobotManager(){}
    
    private:
      // load class
      pluginlib::ClassLoader<control_plugin_base::FootStepPlanner> fsp_loader_;
      pluginlib::ClassLoader<control_plugin_base::WalkingPatternGenerator> wpg_loader_;
      pluginlib::ClassLoader<control_plugin_base::WalkingStabilizationController> wsc_loader_;
      pluginlib::ClassLoader<control_plugin_base::ConvertToJointStates> ctjs_loader_;

      // plugin instances
      std::shared_ptr<control_plugin_base::FootStepPlanner> fsp_ = nullptr;
      std::shared_ptr<control_plugin_base::WalkingPatternGenerator> wpg_ = nullptr;
      std::shared_ptr<control_plugin_base::WalkingStabilizationController> wsc_ = nullptr;
      std::shared_ptr<control_plugin_base::ConvertToJointStates> ctjs_ = nullptr;

      // plugin arguments & return values
      std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr_ = std::make_shared<control_plugin_base::FootStep>();
      std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr_ = std::make_shared<control_plugin_base::WalkingPattern>();
      std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr_ = std::make_shared<control_plugin_base::WalkingStabilization>();
      std::shared_ptr<control_plugin_base::LegJointStatesPattern> leg_joint_states_pat_ptr_ = std::make_shared<control_plugin_base::LegJointStatesPattern>();      

      // step & timer
      void Step_Offline();
      float control_cycle_ = 0;
      uint32_t control_step_ = 0;
      float t_ = 0;
      float walking_time_ = 0;
      uint32_t walking_step_ = 0;
      bool ONLINE_GENERATE_ = false;

      // publisher
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
      std::shared_ptr<sensor_msgs::msg::JointState> pub_joint_states_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
      rclcpp::TimerBase::SharedPtr wall_timer_;

      // publisher (debug mode)
      // rclcpp::Publisher<robot_messages::msg::FootStepPlan>::SharedPtr pub_foot_step_plan_;
      // std::shared_ptr<robot_messages::msg::FootStepPlan> pub_foot_step_plan_msg_ = std::make_shared<robot_messagess::msg::FootStepPlan>();
      // rclcpp::Publisher<robot_messages::msg::WalkingPattern>::SharedPtr pub_walking_pattern_;
      // std::shared_ptr<robot_messages::msg::WalkingPattern> pub_walking_pattern_msg_ = std::make_shared<robot_messagess::msg::WalkingPattern>();
      // rclcpp::Publisher<robot_messages::msg::WalkingStabilization>::SharedPtr pub_walking_stabilization_;
      // std::shared_ptr<robot_messages::msg::WalkingStabilization> pub_walking_stabilization_msg_ = std::make_shared<robot_messagess::msg::WalkingStabilization>();

      // subscriber
      // TODO: ココにFeedbackのSubscriberとCallback関数を定義。
      rclcpp::Subscription<robot_messages::msg::Feedback>::SharedPtr sub_feedback_;
      std::shared_ptr<robot_messages::msg::Feedback> sub_feedback_msg_ = std::make_shared<robot_messages::msg::Feedback>();
      void Feedback_Callback(const robot_messages::msg::Feedback::SharedPtr callback_data);

      // parameters 
        // TODO: 外部から値を取ってきて定義したい子達
        // TODO: 配列の要素数を外から取ってきて、以下を静的配列として定義するプログラムをこのHPPに書きたい
      std::vector<uint8_t> legL_num_;
      std::vector<uint8_t> legR_num_;
      std::vector<int8_t> jointAng_posi_or_nega_legL_; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
      std::vector<int8_t> jointAng_posi_or_nega_legR_;  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
      float T_sup_ = 0;

      // Debug Mode
        // TODO: ココにDebug用のPublisherを定義。Loggerに合わせる。
      bool DebugMode_ = false;
      bool UsingSimulator_ = true;
      std::chrono::system_clock::time_point start_time_;
      std::chrono::system_clock::time_point end_time_;
      double latency_;
      double latency_ctjs_max_ = 0;
      double latency_ctjs_min_ = 9999;
  };
}