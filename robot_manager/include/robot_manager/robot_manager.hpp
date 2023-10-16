#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"

#include "Eigen/Dense"

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

      // step
      void Step();
      uint32_t control_step_ = 0;
      bool ONLINE_GENERATE_ = false;

      // publisher
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
      std::shared_ptr<sensor_msgs::msg::JointState> pub_joint_states_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
      rclcpp::TimerBase::SharedPtr wall_timer_;

      // subscriber
      // TODO: ココにFeedbackのSubscriberとCallback関数を定義。

      // parameters 
        // TODO: 外部から値を取ってきて定義したい子達
        // TODO: 配列の要素数を外から取ってきて、以下を静的配列として定義するプログラムをこのHPPに書きたい
      std::vector<uint8_t> legL_num_;
      std::vector<uint8_t> legR_num_;
      std::vector<int8_t> jointAng_posi_or_nega_legL_; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
      std::vector<int8_t> jointAng_posi_or_nega_legR_;  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)

      // Debug Mode
      // TODO: ココにDebug用のPublisherを定義。Loggerに合わせる。
  };
}