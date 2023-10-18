#include "robot_manager/robot_manager.hpp"

using namespace std::chrono_literals;

namespace robot_manager
{
  void RobotManager::Step() {
// DEBUG: WSCとCTJSの毎Step化が行われるまで、コンストラクタで歩行パターンの生成Pluginを実行する
  // CTJSを毎Step化しないと、10[ms]に収まらない可能性があるため、このような処置をとった。（未測定だけどね）
    // // online or offline generate
    //   // TODO: 既存のFSとWPGに追加するようにしたい。代入での上書きだと既存のパターンが失われるから、オフライン生成しかできない。
    //   // TODO: １回のオンライン生成にかける時間を指定できるようにしたい。10msではなく100msで20歩分の生成とか。別のStepLoopにして、Sleepで周期させる案。
    // if(ONLINE_GENERATE_ == true || control_step_ == 0) {
    //   // Foot_Step_Planner (stack)
    //   // std::cout << "foot step planner" << std::endl;
    //   foot_step_ptr_ = fsp_->foot_step_planner();

    //   // Walking_Pattern_Generator (stack)
    //   // std::cout << "walking pattern generator" << std::endl;
    //   walking_pattern_ptr_ = wpg_->walking_pattern_generator(foot_step_ptr_);
    // }

    // // Walking_Stabilization_Controller (1step)
    // // std::cout << "walking stabilization controller" << std::endl;
    // walking_stabilization_ptr_ = wsc_->walking_stabilization_controller(walking_pattern_ptr_);

    // // Convert_to_Joint_States (1step)
    // // std::cout << "convert to joint states" << std::endl;
    // leg_joint_states_pat_ptr_ = ctjs_->convert_into_joint_states(walking_stabilization_ptr_);
// ここまでDEBUG

    // 歩行パターンを1stepごとPublish
    // TODO: データの重要性からして、ここはServiceのほうがいい気がするんだ。
    // TODO: Pub/Subだから仕方がないが、データの受取ミスが発生する可能性がある。
    auto now_time = rclcpp::Clock().now();
    pub_joint_states_msg_->header.stamp = now_time;
    if(control_step_ < leg_joint_states_pat_ptr_->joint_ang_pat_legL.size()) {  // DEBUG
      for(uint8_t th = 0; th < 6; th++) {
        // CHECKME: WSCとCTJSの型を単位Step用に修正したら、配列に[control_step_]が不要になる。
        pub_joint_states_msg_->position.at(legL_num_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legL[control_step_].at(th) * jointAng_posi_or_nega_legL_.at(th);
        pub_joint_states_msg_->position.at(legR_num_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legR[control_step_].at(th) * jointAng_posi_or_nega_legR_.at(th); 
        pub_joint_states_msg_->velocity.at(legL_num_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legL[control_step_].at(th));
        pub_joint_states_msg_->velocity.at(legR_num_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legR[control_step_].at(th));
      }
    }
    pub_joint_states_->publish(*pub_joint_states_msg_);

    // update
    control_step_++;
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

    // 確実にstep0から送れるようにsleep
    // TODO: Handler側が何かしらのシグナルを出したらPubするようにしたい。
    for(uint16_t step = 0; step < 1000; step++) {
      auto now_time = rclcpp::Clock().now();
      pub_joint_states_msg_->header.stamp = now_time;
      pub_joint_states_->publish(*pub_joint_states_msg_);
      rclcpp::sleep_for(10ms);
    }
    RCLCPP_INFO(this->get_logger(), "Publisher.");

// DEBUG: CTJS内の軌道計算をWPGに移行する前の、Pluginで行けるか否かの確認
  // TODO: WSCとCTJSの毎step化が行けたら、ここは不要。
    //if(ONLINE_GENERATE_ == true || control_step_ == 0) {
      // Foot_Step_Planner (stack)
      // std::cout << "foot step planner" << std::endl;
      foot_step_ptr_ = fsp_->foot_step_planner();

      // Walking_Pattern_Generator (stack)
      // std::cout << "walking pattern generator" << std::endl;
      walking_pattern_ptr_ = wpg_->walking_pattern_generator(foot_step_ptr_);
    //}

    // Walking_Stabilization_Controller (1step)
    // std::cout << "walking stabilization controller" << std::endl;
    walking_stabilization_ptr_ = wsc_->walking_stabilization_controller(walking_pattern_ptr_);

    // Convert_to_Joint_States (1step)
    // std::cout << "convert to joint states" << std::endl;
    leg_joint_states_pat_ptr_ = ctjs_->convert_into_joint_states(walking_stabilization_ptr_, foot_step_ptr_);

    wall_timer_ = this->create_wall_timer(10ms, std::bind(&RobotManager::Step, this));
  }

}