#include "robot_manager/robot_manager.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace robot_manager
{
  void RobotManager::Feedback_Callback(const robot_messages::msg::Feedback::SharedPtr callback_data) {
    sub_feedback_msg_ = callback_data;
  }

  void RobotManager::Step_Offline() {

    if(t_ >= T_sup_ - 0.01) {
      t_ = 0;
      walking_step_++;
    }

    if(control_step_ < walking_pattern_ptr_->cc_cog_pos_ref.size()) { 
      // Walking_Stabilization_Controller (1step)
      // std::cout << "walking stabilization controller" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      walking_stabilization_ptr_ = wsc_->walking_stabilization_controller(walking_pattern_ptr_);
      end_time_ = std::chrono::system_clock::now();
      latency_ = double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000;
      // std::cout << "WSC time [ms] : " << latency_ << std::endl;

      // Convert_to_Joint_States (1step)
      // std::cout << "convert to joint states" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      leg_joint_states_pat_ptr_ = ctjs_->convert_into_joint_states(
        walking_stabilization_ptr_, 
        foot_step_ptr_, 
        walking_step_, 
        control_step_, 
        t_
      );
      end_time_ = std::chrono::system_clock::now();
      latency_ = double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000;
      latency_ctjs_max_ = latency_ > latency_ctjs_max_ ? latency_ : latency_ctjs_max_;
      latency_ctjs_min_ = latency_ < latency_ctjs_min_ ? latency_ : latency_ctjs_min_;
      // std::cout << "CTJS time [ms] : " << latency_ << ", max [ms] : " << latency_ctjs_max_ << ", min [ms] : " << latency_ctjs_min_ <<  std::endl;
      // std::cout << control_step_ << std::endl;
    }

    // TODO: データの重要性からして、ここはServiceのほうがいい気がするんだ。
    // TODO: Pub/Subだから仕方がないが、データの受取ミスが発生する可能性がある。
    auto now_time = rclcpp::Clock().now();
    pub_joint_states_msg_->header.stamp = now_time;
    if(control_step_ < walking_pattern_ptr_->cc_cog_pos_ref.size()) {
      for(uint8_t th = 0; th < 6; th++) {
        // CHECKME: WSCとCTJSの型を単位Step用に修正したら、配列に[control_step_]が不要になる。
        pub_joint_states_msg_->position.at(legL_num_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legL.at(th) * jointAng_posi_or_nega_legL_.at(th);
        pub_joint_states_msg_->position.at(legR_num_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legR.at(th) * jointAng_posi_or_nega_legR_.at(th); 
        pub_joint_states_msg_->velocity.at(legL_num_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legL.at(th));
        pub_joint_states_msg_->velocity.at(legR_num_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legR.at(th));
      }
      // std::cout << control_step_ << std::endl;
    }
    pub_joint_states_->publish(*pub_joint_states_msg_);

    // update
    control_step_++;
    t_ += control_cycle_;
    walking_time_ += control_cycle_;
  }

  RobotManager::RobotManager(
    const rclcpp::NodeOptions& options
  ) : Node("RobotManager", options),
    // load class
    fsp_loader_("robot_manager", "control_plugin_base::FootStepPlanner"),
    wpg_loader_("robot_manager", "control_plugin_base::WalkingPatternGenerator"),
    wsc_loader_("robot_manager", "control_plugin_base::WalkingStabilizationController"),
    ctjs_loader_("robot_manager", "control_plugin_base::ConvertToJointStates") {
    // plugins
    try {
      // create instances
      fsp_ = fsp_loader_.createSharedInstance("foot_step_planner::Default_FootStepPlanner");
      wpg_ = wpg_loader_.createSharedInstance("walking_pattern_generator::WPG_LinearInvertedPendulumModel");
      wsc_ = wsc_loader_.createSharedInstance("walking_stabilization_controller::Default_WalkingStabilizationController");
      ctjs_ = ctjs_loader_.createSharedInstance("convert_to_joint_states::Default_ConvertToJointStates");
    }
    catch(pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }

    // publisher
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    //subscription
    using namespace std::placeholders;
    sub_feedback_ = this->create_subscription<robot_messages::msg::Feedback>("feedback", 10, std::bind(&RobotManager::Feedback_Callback, this, _1));

    // setting /joint_states pub_joint_states_msg_
    // TODO: これはParameterServerからやりたい。
    std::vector<std::string> name = {
        "head_pan",
        "head_tilt",
        "l_sho_pitch",
        "l_sho_roll",
        "l_el",
        "r_sho_pitch",
        "r_sho_roll",
        "r_el",
        "l_hip_yaw",
        "l_hip_roll",
        "l_hip_pitch",
        "l_knee",
        "l_ank_pitch",
        "l_ank_roll",
        "r_hip_yaw",
        "r_hip_roll",
        "r_hip_pitch",
        "r_knee",
        "r_ank_pitch",
        "r_ank_roll"
    };
    legL_num_ = { 8,  9, 10, 11, 12, 13};
    legR_num_ = {14, 15, 16, 17, 18, 19};
    jointAng_posi_or_nega_legR_ = {-1, -1, 1, 1, -1, 1};  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
    jointAng_posi_or_nega_legL_ = {-1, -1, -1, -1, 1, 1}; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
    pub_joint_states_msg_->name.resize(20);
    pub_joint_states_msg_->position.resize(20);
    pub_joint_states_msg_->velocity.resize(20);
    for(uint8_t th = 0; th < 20; th++) {
      pub_joint_states_msg_->name.at(th) = name.at(th);
      pub_joint_states_msg_->position.at(th) = 0.0;
      pub_joint_states_msg_->velocity.at(th) = 0.0;
    }

    // Setting parameters
      // TODO: ParameterServerとかから得た値を使いたい
    ONLINE_GENERATE_ = false;
    DebugMode_ = false;
    UsingSimulator_ = true;

    // timer
    control_cycle_ = 0.01;  // 10[ms]
    T_sup_ = 0.8;  // 歩行周期

    // 確実にstep0から送れるようにsleep
      // TODO: Handler側が何かしらのシグナルを出したらPubするようにしたい。
    if(UsingSimulator_ == true) {
      for(uint16_t step = 0; step < 1000; step++) {
        auto now_time = rclcpp::Clock().now();
        pub_joint_states_msg_->header.stamp = now_time;
        pub_joint_states_->publish(*pub_joint_states_msg_);
        rclcpp::sleep_for(10ms);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Start Control Cycle.");

    // オフラインパターン生成
    if(ONLINE_GENERATE_ == false) {
      // Foot_Step_Planner (stack)
      // std::cout << "foot step planner" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      foot_step_ptr_ = fsp_->foot_step_planner();
      end_time_ = std::chrono::system_clock::now();
      latency_ = double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000;
      // std::cout << "FSP time [ms] : " << latency_ << std::endl;

      // Walking_Pattern_Generator (stack)
      // std::cout << "walking pattern generator" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      walking_pattern_ptr_ = wpg_->walking_pattern_generator(foot_step_ptr_);
      end_time_ = std::chrono::system_clock::now();
      latency_ = double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000;
      // std::cout << "WPG time [ms] : " << latency_ << std::endl;
    }

    wall_timer_ = this->create_wall_timer(10ms, std::bind(&RobotManager::Step_Offline, this));
  }

}