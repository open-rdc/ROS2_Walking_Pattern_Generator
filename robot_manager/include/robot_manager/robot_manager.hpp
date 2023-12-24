#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_messages/msg/feedback.hpp"
#include "robot_messages/msg/foot_step_record.hpp"
#include "robot_messages/msg/walking_pattern_record.hpp"
#include "robot_messages/msg/walking_stabilization_record.hpp"
#include "robot_messages/msg/joint_state_record.hpp"

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
      ~RobotManager();
    
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

      // client parameters
      rclcpp::Node::SharedPtr node_ptr_;
      std::shared_ptr<rclcpp::SyncParametersClient> client_param_;

      // param: mode_switch
      bool ONLINE_OR_OFFLINE_GENERATE_ = false;
      bool DEBUG_MODE_ = false;
      bool USING_SIMULATOR_ = false;

      // param: robot_description
      std::string ROBOT_NAME_;

      // param: control
      double CONTROL_CYCLE_ = 0;
      double WALKING_CYCLE_ = 0;

      // param: limb
      std::string LEFT_LEG_NAME_;
      std::string RIGHT_LEG_NAME_;
      std::vector<long> LEFT_LEG_JOINT_NUMBERS_;
      std::vector<long> RIGHT_LEG_JOINT_NUMBERS_;

      // param: name_lists
      std::vector<std::string> ALL_JOINT_NAMES_WITHOUT_FIXED_;

      // step & timer
      void Step_Offline();  // debug mode抜き
      void Step_Offline_DebugMode();  // debug mode用
      // float control_cycle_ = 0;
      uint32_t control_step_ = 0;
      float t_ = 0;
      float walking_time_ = 0;
      uint32_t walking_step_ = 0;
      // bool ONLINE_GENERATE_ = false;

      // publisher
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
      std::shared_ptr<sensor_msgs::msg::JointState> pub_joint_states_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
      rclcpp::TimerBase::SharedPtr wall_timer_;

      // publisher (debug mode)
      rclcpp::Publisher<robot_messages::msg::FootStepRecord>::SharedPtr pub_foot_step_plan_record_;
      std::shared_ptr<robot_messages::msg::FootStepRecord> pub_foot_step_plan_record_msg_ = std::make_shared<robot_messages::msg::FootStepRecord>();
      rclcpp::Publisher<robot_messages::msg::WalkingPatternRecord>::SharedPtr pub_walking_pattern_record_;
      std::shared_ptr<robot_messages::msg::WalkingPatternRecord> pub_walking_pattern_record_msg_ = std::make_shared<robot_messages::msg::WalkingPatternRecord>();
      rclcpp::Publisher<robot_messages::msg::WalkingStabilizationRecord>::SharedPtr pub_walking_stabilization_record_;
      std::shared_ptr<robot_messages::msg::WalkingStabilizationRecord> pub_walking_stabilization_record_msg_ = std::make_shared<robot_messages::msg::WalkingStabilizationRecord>();
      rclcpp::Publisher<robot_messages::msg::JointStateRecord>::SharedPtr pub_joint_state_record_;
      std::shared_ptr<robot_messages::msg::JointStateRecord> pub_joint_state_record_msg_ = std::make_shared<robot_messages::msg::JointStateRecord>();

      // subscriber
      // TODO: ココにFeedbackのSubscriberとCallback関数を定義。
      rclcpp::Subscription<robot_messages::msg::Feedback>::SharedPtr sub_feedback_;
      std::shared_ptr<robot_messages::msg::Feedback> sub_feedback_msg_ = std::make_shared<robot_messages::msg::Feedback>();
      void Feedback_Callback(const robot_messages::msg::Feedback::SharedPtr callback_data);

      // parameters 
        // TODO: 外部から値を取ってきて定義したい子達
        // TODO: 配列の要素数を外から取ってきて、以下を静的配列として定義するプログラムをこのHPPに書きたい
      // std::vector<uint8_t> legL_num_;
      // std::vector<uint8_t> legR_num_;
      // std::vector<int8_t> jointAng_posi_or_nega_legL_; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
      // std::vector<int8_t> jointAng_posi_or_nega_legR_;  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
      // float T_sup_ = 0;

      // Debug Mode
        // TODO: ココにDebug用のPublisherを定義。Loggerに合わせる。
      // bool DebugMode_ = false;
      // bool UsingSimulator_ = true;

      // Debug Mode: recording time
      std::chrono::system_clock::time_point start_time_;
      std::chrono::system_clock::time_point end_time_;
      double latency_;
      double latency_fsp_offline_;
      double latency_wpg_offline_;
      std::vector<double> all_latency_wsc_;
      double latency_wsc_ave_ = 0;
      double latency_wsc_med_ = 0;
      double latency_wsc_max_ = 0;
      double latency_wsc_min_ = 9999;
      std::vector<double> all_latency_ctjs_;
      double latency_ctjs_ave_ = 0;
      double latency_ctjs_med_ = 0;
      double latency_ctjs_max_ = 0;
      double latency_ctjs_min_ = 9999;
      std::chrono::system_clock::time_point now_time_;
      std::chrono::system_clock::time_point before_time_;
      std::vector<double> all_latency_pub_joint_states_;
      double latency_pub_joint_states_ave_ = 0;
      double latency_pub_joint_states_med_ = 0;
      double latency_pub_joint_states_max_ = 0;
      double latency_pub_joint_states_min_ = 9999;
  };
}