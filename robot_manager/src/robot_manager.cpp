#include "robot_manager/robot_manager.hpp"

#include "Eigen/Dense"

using namespace Eigen;

namespace robot_manager
{
  void Step() {
    // 歩行パターンを1stepごとPublish
    // TODO: データの重要性からして、ここはServiceのほうがいい気がするんだ。
    // TODO: Pub/Subだから仕方がないが、データの受取ミスが発生する。
    for(int step = 0; step < int(WalkingPattern_Pos_legL_.size()); step++) {
      // pub_msg->q_next_leg_l = WalkingPattern_Pos_legL_[step];
      // pub_msg->q_next_leg_r = WalkingPattern_Pos_legR_[step];
      // pub_msg->dq_next_leg_l = WalkingPattern_Vel_legL_[step];
      // pub_msg->dq_next_leg_r = WalkingPattern_Vel_legR_[step];
      // pub_msg->counter = step;
      // RCLCPP_INFO(this->get_logger(), "publish...: [ %d ]", pub_msg->counter);
      auto now_time = rclcpp::Clock().now();
      pub_msg->header.stamp = now_time;
      for(uint8_t th = 0; th < 6; th++) {
        pub_msg->position.at(legL_num.at(th)) = WalkingPattern_Pos_legL_.at(step).at(th) * jointAng_posi_or_nega_legL_.at(th);
        pub_msg->position.at(legR_num.at(th)) = WalkingPattern_Pos_legR_.at(step).at(th) * jointAng_posi_or_nega_legR_.at(th); 
        pub_msg->velocity.at(legL_num.at(th)) = std::abs(WalkingPattern_Vel_legL_.at(step).at(th));
        pub_msg->velocity.at(legR_num.at(th)) = std::abs(WalkingPattern_Vel_legR_.at(step).at(th));
      }
      pub_walking_pattern_->publish(*pub_msg);
      rclcpp::sleep_for(10ms);
    }
  }

  RobotManager::RobotManager() {
    // CHECKME
    pub_walking_pattern_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 5);
    auto pub_msg = std::make_shared<sensor_msgs::msg::JointState>();

    // DEBUG: parameter setting
    DEBUG_ParameterSetting();

    // 歩行パターン生成
    WalkingPatternGenerate();
    RCLCPP_INFO(this->get_logger(), "Create Walking Pattern.");

    // DEBUG: etting /joint_states pub_msg
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
    std::array<uint8_t, 6> legL_num = { 8,  9, 10, 11, 12, 13};
    std::array<uint8_t, 6> legR_num = {14, 15, 16, 17, 18, 19};
    std::array<int8_t, 6> jointAng_posi_or_nega_legR_ = {-1, -1, 1, 1, -1, 1};  // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (right leg)
    std::array<int8_t, 6> jointAng_posi_or_nega_legL_ = {-1, -1, -1, -1, 1, 1}; // positive & negative. Changed from riht-handed system to specification of ROBOTIS OP2 of Webots. (left leg)
    pub_msg->name.resize(20);
    pub_msg->position.resize(20);
    pub_msg->velocity.resize(20);
    for(uint8_t th = 0; th < 20; th++) {
      pub_msg->name.at(th) = name.at(th);
      pub_msg->position.at(th) = 0.0;
      pub_msg->velocity.at(th) = 0.0;
    }

    // 確実にstep0から送れるようにsleep
    // TODO: Handler側が何かしらのシグナルを出したらPubするようにしたい。
    for(uint16_t step = 0; step < 1000; step++) {
      auto now_time = rclcpp::Clock().now();
      pub_msg->header.stamp = now_time;
      pub_walking_pattern_->publish(*pub_msg);
      rclcpp::sleep_for(10ms);
    }
    RCLCPP_INFO(this->get_logger(), "Publisher.");


  }
}

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<control_plugin_base::WalkingPatternGenerator> wpg_loader("robot_manager", "control_plugin_base::WalkingPatternGenerator");
  pluginlib::ClassLoader<control_plugin_base::FootStepPlanner> fsp_loader("robot_manager", "control_plugin_base::FootStepPlanner");
  pluginlib::ClassLoader<control_plugin_base::WalkingStabilizationController> wsc_loader("robot_manager", "control_plugin_base::WalkingStabilizationController");
  pluginlib::ClassLoader<control_plugin_base::ConvertToJointStates> ctjs_loader("robot_manager", "control_plugin_base::ConvertToJointStates");
  // pluginlib::ClassLoader<control_plugin_base::ForwardKinematics> fk_loader("robot_manager", "control_plugin_base::ForwardKinematics");
  // pluginlib::ClassLoader<control_plugin_base::InverseKinematics> ik_loader("robot_manager", "control_plugin_base::InverseKinematics");
  // pluginlib::ClassLoader<control_plugin_base::Jacobian> jac_loader("robot_manager", "control_plugin_base::Jacobian");

  try
  {
    std::shared_ptr<control_plugin_base::WalkingPatternGenerator> wpg = wpg_loader.createSharedInstance("walking_pattern_generator::WPG_LinearInvertedPendulumModel");
    std::shared_ptr<control_plugin_base::FootStepPlanner> fsp = fsp_loader.createSharedInstance("foot_step_planner::Default_FootStepPlanner");
    std::shared_ptr<control_plugin_base::WalkingStabilizationController> wsc = wsc_loader.createSharedInstance("walking_stabilization_controller::Default_WalkingStabilizationController");
    std::shared_ptr<control_plugin_base::ConvertToJointStates> ctjs = ctjs_loader.createSharedInstance("convert_to_joint_states::Default_ConvertToJointStates");
    // std::shared_ptr<control_plugin_base::ForwardKinematics> fk = fk_loader.createSharedInstance("kinematics::Default_ForwardKinematics");
    // std::shared_ptr<control_plugin_base::InverseKinematics> ik = ik_loader.createSharedInstance("kinematics::Default_InverseKinematics");
    // std::shared_ptr<control_plugin_base::Jacobian> jac = jac_loader.createSharedInstance("kinematics::Default_Jacobian");

// Foot_Step_Planner
    std::cout << "foot step planner" << std::endl;
    std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr = fsp->foot_step_planner();

// Walking_Pattern_Generator
    std::cout << "walking pattern generator" << std::endl;
    std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = wpg->walking_pattern_generator(foot_step_ptr);

// Walking_Stabilization_Controller
    std::cout << "walking stabilization controller" << std::endl;
    std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr = wsc->walking_stabilization_controller(walking_pattern_ptr);

// Convert_to_Joint_States
    std::cout << "convert to joint states" << std::endl;
    std::shared_ptr<control_plugin_base::LegJointStatesPattern> leg_joint_states_pat_ptr = ctjs->convert_into_joint_states(walking_stabilization_ptr);

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
    
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("ERROR!!: %s\n", ex.what());
  }
  
  return 0;
}