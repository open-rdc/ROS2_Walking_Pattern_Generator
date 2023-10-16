#include "robot_manager/robot_manager.hpp"

using namespace std::chrono_literals;

namespace robot_manager
{
  void RobotManager::Step() {

    // online or offline generate
      // TODO: 既存のFSとWPGに追加するようにしたい。代入での上書きだと既存のパターンが失われるから、オフライン生成しかできない。
      // TODO: １回のオンライン生成にかける時間を指定できるようにしたい。10msではなく100msで20歩分の生成とか。別のStepLoopにして、Sleepで周期させる案。
    if(ONLINE_GENERATE_ == true || control_step_ == 0) {
      // Foot_Step_Planner (stack)
      // std::cout << "foot step planner" << std::endl;
      foot_step_ptr_ = fsp_->foot_step_planner();

      // Walking_Pattern_Generator (stack)
      // std::cout << "walking pattern generator" << std::endl;
      walking_pattern_ptr_ = wpg_->walking_pattern_generator(foot_step_ptr_);
    }

    // Walking_Stabilization_Controller (1step)
    // std::cout << "walking stabilization controller" << std::endl;
    walking_stabilization_ptr_ = wsc_->walking_stabilization_controller(walking_pattern_ptr_);

    // Convert_to_Joint_States (1step)
    // std::cout << "convert to joint states" << std::endl;
    leg_joint_states_pat_ptr_ = ctjs_->convert_into_joint_states(walking_stabilization_ptr_);

    // 歩行パターンを1stepごとPublish
    // TODO: データの重要性からして、ここはServiceのほうがいい気がするんだ。
    // TODO: Pub/Subだから仕方がないが、データの受取ミスが発生する可能性がある。
    auto now_time = rclcpp::Clock().now();
    pub_joint_states_msg_->header.stamp = now_time;
    for(uint8_t th = 0; th < 6; th++) {
      // CHECKME: WSCとCTJSの型を単位Step用に修正したら、配列に[0]が不要になる。
      pub_joint_states_msg_->position.at(legL_num_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legL[0].at(th) * jointAng_posi_or_nega_legL_.at(th);
      pub_joint_states_msg_->position.at(legR_num_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legR[0].at(th) * jointAng_posi_or_nega_legR_.at(th); 
      pub_joint_states_msg_->velocity.at(legL_num_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legL[0].at(th));
      pub_joint_states_msg_->velocity.at(legR_num_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legR[0].at(th));
    }
    pub_joint_states_->publish(*pub_joint_states_msg_);

    // update
    control_step_++;
  }

  RobotManager::RobotManager(
    const rclcpp::NodeOptions& options
  ) : Node("RobotManager", options) {
    // plugins
    try {
      // load classes
      pluginlib::ClassLoader<control_plugin_base::WalkingPatternGenerator> wpg_loader("robot_manager", "control_plugin_base::WalkingPatternGenerator");
      pluginlib::ClassLoader<control_plugin_base::FootStepPlanner> fsp_loader("robot_manager", "control_plugin_base::FootStepPlanner");
      pluginlib::ClassLoader<control_plugin_base::WalkingStabilizationController> wsc_loader("robot_manager", "control_plugin_base::WalkingStabilizationController");
      pluginlib::ClassLoader<control_plugin_base::ConvertToJointStates> ctjs_loader("robot_manager", "control_plugin_base::ConvertToJointStates");
      // create instances
      fsp_ = fsp_loader.createSharedInstance("foot_step_planner::Default_FootStepPlanner");
      wpg_ = wpg_loader.createSharedInstance("walking_pattern_generator::WPG_LinearInvertedPendulumModel");
      wsc_ = wsc_loader.createSharedInstance("walking_stabilization_controller::Default_WalkingStabilizationController");
      ctjs_ = ctjs_loader.createSharedInstance("convert_to_joint_states::Default_ConvertToJointStates");
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


  }
}

// int main(int argc, char** argv) {
//   (void) argc;
//   (void) argv;

//   // pluginlib::ClassLoader<control_plugin_base::ForwardKinematics> fk_loader("robot_manager", "control_plugin_base::ForwardKinematics");
//   // pluginlib::ClassLoader<control_plugin_base::InverseKinematics> ik_loader("robot_manager", "control_plugin_base::InverseKinematics");
//   // pluginlib::ClassLoader<control_plugin_base::Jacobian> jac_loader("robot_manager", "control_plugin_base::Jacobian");

//   try
//   {
//     fsp_ = fsp_loader.createSharedInstance("foot_step_planner::Default_FootStepPlanner");
//     wpg_ = wpg_loader.createSharedInstance("walking_pattern_generator::WPG_LinearInvertedPendulumModel");
//     wsc_ = wsc_loader.createSharedInstance("walking_stabilization_controller::Default_WalkingStabilizationController");
//     ctjs_ = ctjs_loader.createSharedInstance("convert_to_joint_states::Default_ConvertToJointStates");
//     // std::shared_ptr<control_plugin_base::ForwardKinematics> fk = fk_loader.createSharedInstance("kinematics::Default_ForwardKinematics");
//     // std::shared_ptr<control_plugin_base::InverseKinematics> ik = ik_loader.createSharedInstance("kinematics::Default_InverseKinematics");
//     // std::shared_ptr<control_plugin_base::Jacobian> jac = jac_loader.createSharedInstance("kinematics::Default_Jacobian");

// // Foot_Step_Planner
//     std::cout << "foot step planner" << std::endl;
//     foot_step_ptr_ = fsp_->foot_step_planner();

// // Walking_Pattern_Generator
//     std::cout << "walking pattern generator" << std::endl;
//     walking_pattern_ptr_ = wpg_->walking_pattern_generator(foot_step_ptr);

// // Walking_Stabilization_Controller
//     std::cout << "walking stabilization controller" << std::endl;
//     walking_stabilization_ptr_ = wsc_->walking_stabilization_controller(walking_pattern_ptr);

// // Convert_to_Joint_States
//     std::cout << "convert to joint states" << std::endl;
//     leg_joint_states_pat_ptr_ = ctjs_->convert_into_joint_states(walking_stabilization_ptr);
//   }
//   catch(pluginlib::PluginlibException& ex)
//   {
//     printf("ERROR!!: %s\n", ex.what());
//   }
  
//   return 0;
// }

    // // DEBUG: IK&FKの動作確認
    // std::array<double, 6> legR_joint_ang;
    // std::shared_ptr<control_plugin_base::LegStates_ToIK> legR_states_IK_ptr = std::make_shared<control_plugin_base::LegStates_ToIK>();  // 変数名、FKと間違えやすい。
    // legR_states_IK_ptr->end_eff_pos = {-0.03, -0.1, -0.10};
    // legR_states_IK_ptr->end_eff_rot = Matrix3d{ {1, 0, 0},
    //                                             {0, 1, 0},
    //                                             {0, 0, 1}
    // };
    // legR_states_IK_ptr->link_len = {Vector3d{-0.005,-0.037,0},
    //                                 Vector3d{0,0,0},
    //                                 Vector3d{0,0,0},
    //                                 Vector3d{0,0,-0.093},
    //                                 Vector3d{0,0,-0.093},
    //                                 Vector3d{0,0,0},
    //                                 Vector3d{0,0,0}
    // };
    // ik->inverse_kinematics(legR_states_IK_ptr, legR_joint_ang);
    // std::cout << "[DEBUG]: [IK]: end_pos_{" << legR_states_IK_ptr->end_eff_pos.transpose() << "}, joint_ang_{"  << legR_joint_ang[0] << ", "
    //                                                                                                             << legR_joint_ang[1] << ", "
    //                                                                                                             << legR_joint_ang[2] << ", "
    //                                                                                                             << legR_joint_ang[3] << ", "
    //                                                                                                             << legR_joint_ang[4] << ", "
    //                                                                                                             << legR_joint_ang[5] << "} "<< std::endl; 

    // Eigen::Vector3d legR_end_eff_pos;
    // std::shared_ptr<control_plugin_base::LegStates_ToFK> legR_states_FK_ptr = std::make_shared<control_plugin_base::LegStates_ToFK>();
    // legR_states_FK_ptr->joint_ang = legR_joint_ang;  // rad
    // legR_states_FK_ptr->link_len = legR_states_IK_ptr->link_len;
    // // legR_states_FK_ptr->joint_point = 6;
    // fk->forward_kinematics(legR_states_FK_ptr, legR_end_eff_pos);
    // std::cout << "[DEBUG]: [FK]: joint_point " << 6 << ", end_effecter_position: { "<< legR_end_eff_pos[0]
    //                                                                         << ", " << legR_end_eff_pos[1] 
    //                                                                         << ", " << legR_end_eff_pos[2] << " }" << std::endl;

    // // DEBUG: Jacobianの動作確認
    // std::shared_ptr<control_plugin_base::LegStates_ToJac> legR_states_jac_ptr = std::make_shared<control_plugin_base::LegStates_ToJac>();
    // legR_states_jac_ptr->joint_ang = legR_states_FK_ptr->joint_ang;
    // legR_states_jac_ptr->link_len = legR_states_IK_ptr->link_len;
    // legR_states_jac_ptr->unit_vec = {  // legR joint unit vector
    //   Vector3d(0, 0, 1),
    //   Vector3d(1, 0, 0),
    //   Vector3d(0, 1, 0),
    //   Vector3d(0, 1, 0),
    //   Vector3d(0, 1, 0),
    //   Vector3d(1, 0, 0)
    // };
    // Matrix<double, 6, 6> legR_jacobian;
    // jac->jacobian(legR_states_jac_ptr, legR_jacobian);  /// (引数, 返り値)
    // std::cout << legR_jacobian << std::endl;