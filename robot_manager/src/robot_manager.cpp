#include "robot_manager/robot_manager.hpp"
#include <chrono>
#include <fstream>
#include <algorithm>

#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_messages/msg/feedback.hpp"
#include "robot_messages/msg/foot_step_record.hpp"
#include "robot_messages/msg/walking_pattern_record.hpp"
#include "robot_messages/msg/walking_stabilization_record.hpp"
#include "robot_messages/msg/joint_state_record.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;


namespace robot_manager
{
  void RobotManager::Feedback_Callback(const robot_messages::msg::Feedback::SharedPtr callback_data) {
    sub_feedback_msg_ = callback_data;
  }

  void RobotManager::Step_Offline() {

    if(t_ >= WALKING_CYCLE_ - 0.01) {
      t_ = 0;
      walking_step_++;
    }

    if(control_step_ < walking_pattern_ptr_->cc_cog_pos_ref.size()) { 

      if(DEBUG_MODE_ == true) {
        // pub foot_step (1step)
        pub_foot_step_plan_record_msg_->step_count = control_step_;
        pub_foot_step_plan_record_msg_->foot_step_pos = foot_step_ptr_->foot_pos[walking_step_];
        pub_foot_step_plan_record_->publish(*pub_foot_step_plan_record_msg_);
        pub_walking_pattern_record_msg_->step_count = control_step_;

        // pub walking_pattern (1step)
        pub_walking_pattern_record_msg_->cc_cog_pos_ref = walking_pattern_ptr_->cc_cog_pos_ref[control_step_];
        pub_walking_pattern_record_msg_->cc_cog_vel_ref = walking_pattern_ptr_->cc_cog_vel_ref[control_step_];
        pub_walking_pattern_record_msg_->wc_foot_land_pos_ref = walking_pattern_ptr_->wc_foot_land_pos_ref[walking_step_];
        pub_walking_pattern_record_->publish(*pub_walking_pattern_record_msg_);
      }

      // Walking_Stabilization_Controller (1step)
      // std::cout << "walking stabilization controller" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      walking_stabilization_ptr_ = wsc_->walking_stabilization_controller(walking_pattern_ptr_);  // TODO: vector<array>で返さずに、1step分のarrayのみを返すようにするべき。
      end_time_ = std::chrono::system_clock::now();
      all_latency_wsc_.push_back(double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000);
      // std::cout << "WSC time [ms] : " << latency_ << std::endl;

      if(DEBUG_MODE_ == true) {
        //pub walking_stabilization (1step)
        pub_walking_stabilization_record_msg_->step_count = control_step_;
        pub_walking_stabilization_record_msg_->cog_pos_fix = walking_stabilization_ptr_->cog_pos_fix[control_step_];
        pub_walking_stabilization_record_msg_->cog_vel_fix = walking_stabilization_ptr_->cog_vel_fix[control_step_];
        pub_walking_stabilization_record_msg_->zmp_pos_fix = walking_stabilization_ptr_->zmp_pos_fix[walking_step_];
        pub_walking_stabilization_record_->publish(*pub_walking_stabilization_record_msg_);
      }

      // Convert_to_Joint_States (1step)
      // std::cout << "convert to joint states" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      leg_joint_states_pat_ptr_ = ctjs_->convert_to_joint_states(
        walking_stabilization_ptr_, 
        foot_step_ptr_, 
        walking_step_, 
        control_step_, 
        t_
      );
      end_time_ = std::chrono::system_clock::now();
      all_latency_ctjs_.push_back(double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000);
      // latency_ctjs_max_ = latency_ > latency_ctjs_max_ ? latency_ : latency_ctjs_max_;
      // latency_ctjs_min_ = latency_ < latency_ctjs_min_ ? latency_ : latency_ctjs_min_;
      // std::cout << "CTJS time [ms] : " << latency_ << ", max [ms] : " << latency_ctjs_max_ << ", min [ms] : " << latency_ctjs_min_ <<  std::endl;
      // std::cout << control_step_ << std::endl;

      if(DEBUG_MODE_ == true) {
        // pub joint_states (1step)
        pub_joint_state_record_msg_->step_count = control_step_;
        pub_joint_state_record_msg_->joint_ang_leg_l = leg_joint_states_pat_ptr_->joint_ang_pat_legL;
        pub_joint_state_record_msg_->joint_ang_leg_r = leg_joint_states_pat_ptr_->joint_ang_pat_legR;
        pub_joint_state_record_msg_->joint_vel_leg_l = leg_joint_states_pat_ptr_->joint_vel_pat_legL;
        pub_joint_state_record_msg_->joint_vel_leg_r = leg_joint_states_pat_ptr_->joint_vel_pat_legR;
        pub_joint_state_record_->publish(*pub_joint_state_record_msg_);
      }
    }

    auto now_time = rclcpp::Clock().now();
    pub_joint_states_msg_->header.stamp = now_time;
    if(control_step_ < walking_pattern_ptr_->cc_cog_pos_ref.size()) {
      for(uint8_t th = 0; th < 6; th++) {
        pub_joint_states_msg_->position.at(LEFT_LEG_JOINT_NUMBERS_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legL.at(th);
        pub_joint_states_msg_->position.at(RIGHT_LEG_JOINT_NUMBERS_.at(th)) = leg_joint_states_pat_ptr_->joint_ang_pat_legR.at(th); 
        pub_joint_states_msg_->velocity.at(LEFT_LEG_JOINT_NUMBERS_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legL.at(th));
        pub_joint_states_msg_->velocity.at(RIGHT_LEG_JOINT_NUMBERS_.at(th)) = std::abs(leg_joint_states_pat_ptr_->joint_vel_pat_legR.at(th));
      }
      // std::cout << control_step_ << std::endl;
    }
    pub_joint_states_->publish(*pub_joint_states_msg_);
    if(control_step_ < walking_pattern_ptr_->cc_cog_pos_ref.size()) { 
      if(control_step_ == 0) {
        now_time_ = std::chrono::system_clock::now();
        before_time_ = std::chrono::system_clock::now();
        //all_latency_pub_joint_states_.push_back(double(std::chrono::duration_cast<std::chrono::microseconds>(now_time_ - before_time_).count()) / 1000);
      }
      else {
        now_time_ = std::chrono::system_clock::now();
        all_latency_pub_joint_states_.push_back(double(std::chrono::duration_cast<std::chrono::microseconds>(now_time_ - before_time_).count()) / 1000);
        before_time_ = now_time_;
      }
    }

    // update
    control_step_++;
    t_ += CONTROL_CYCLE_;
    walking_time_ += CONTROL_CYCLE_;
  }

  RobotManager::RobotManager(
    const rclcpp::NodeOptions& options
  ) : Node("RobotManager", options),
    // load class
    fsp_loader_("robot_manager", "control_plugin_base::FootStepPlanner"),
    wpg_loader_("robot_manager", "control_plugin_base::WalkingPatternGenerator"),
    wsc_loader_("robot_manager", "control_plugin_base::WalkingStabilizationController"),
    ctjs_loader_("robot_manager", "control_plugin_base::ConvertToJointStates") 
  {
    rclcpp::sleep_for(2s);  // ParameterServerの立ち上げの完了を見越した待機。無しだと、ParameterServerが恐らく起動完了していない。

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

    // client parrameters
    node_ptr_ = rclcpp::Node::make_shared("RobotManager");
    client_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "RobotParameterServer");
    // param: mode_switch
    ONLINE_OR_OFFLINE_GENERATE_ = client_param_->get_parameter<bool>("mode_switch.on_or_offline_pattern_generate");
    DEBUG_MODE_ = client_param_->get_parameter<bool>("mode_switch.debug_mode");
    USING_SIMULATOR_ = client_param_->get_parameter<bool>("mode_switch.using_simulator");
    // param: robot_description
    ROBOT_NAME_ = client_param_->get_parameter<std::string>("robot_description.robot_name");
    // param: control
    CONTROL_CYCLE_ = client_param_->get_parameter<double>("control_times.control_cycle");
    WALKING_CYCLE_ = client_param_->get_parameter<double>("control_times.walking_cycle");
    // param: limb
    LEFT_LEG_NAME_ = client_param_->get_parameter<std::string>(ROBOT_NAME_+"_limb.limb_names.left_leg");
    RIGHT_LEG_NAME_ = client_param_->get_parameter<std::string>(ROBOT_NAME_+"_limb.limb_names.right_leg");
    LEFT_LEG_JOINT_NUMBERS_ = client_param_->get_parameter<std::vector<long>>(ROBOT_NAME_+"_limb.limb_without_fixed_joints."+LEFT_LEG_NAME_+".joint_numbers");
    RIGHT_LEG_JOINT_NUMBERS_ = client_param_->get_parameter<std::vector<long>>(ROBOT_NAME_+"_limb.limb_without_fixed_joints."+RIGHT_LEG_NAME_+".joint_numbers");
    // param: name_lists
    ALL_JOINT_NAMES_WITHOUT_FIXED_ = client_param_->get_parameter<std::vector<std::string>>(ROBOT_NAME_+"_name_lists.all_names_without_fixed_joints.all_joint_names");

    // publisher
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    if(DEBUG_MODE_ == true) {
      pub_foot_step_plan_record_ = this->create_publisher<robot_messages::msg::FootStepRecord>("foot_step", 10);
      pub_walking_pattern_record_ = this->create_publisher<robot_messages::msg::WalkingPatternRecord>("walking_pattern", 10);
      pub_walking_stabilization_record_ = this->create_publisher<robot_messages::msg::WalkingStabilizationRecord>("walking_stabilization", 10);
      pub_joint_state_record_ = this->create_publisher<robot_messages::msg::JointStateRecord>("joint_states_record", 10);
    }
    //subscription
    sub_feedback_ = this->create_subscription<robot_messages::msg::Feedback>("feedback", 10, std::bind(&RobotManager::Feedback_Callback, this, _1));
    
    pub_joint_states_msg_->name.resize(20);
    pub_joint_states_msg_->position.resize(20);
    pub_joint_states_msg_->velocity.resize(20);
    for(uint8_t th = 0; th < 20; th++) {
      pub_joint_states_msg_->name.at(th) = ALL_JOINT_NAMES_WITHOUT_FIXED_.at(th);
      pub_joint_states_msg_->position.at(th) = 0.0;
      pub_joint_states_msg_->velocity.at(th) = 0.0;
    }

    // 確実にstep0から送れるようにsleep
      // TODO: Handler側が何かしらのシグナルを出したらPubするようにしたい。
    if(USING_SIMULATOR_ == true) {
      for(uint16_t step = 0; step < 800; step++) {
        auto now_time = rclcpp::Clock().now();
        pub_joint_states_msg_->header.stamp = now_time;
        pub_joint_states_->publish(*pub_joint_states_msg_);
        rclcpp::sleep_for(10ms);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Start Control Cycle.");

    // Offline Pattern Generate
    if(ONLINE_OR_OFFLINE_GENERATE_ == false) {
      // Foot_Step_Planner (stack)
      // std::cout << "foot step planner" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      foot_step_ptr_ = fsp_->foot_step_planner();
      end_time_ = std::chrono::system_clock::now();
      latency_fsp_offline_ = double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000;
      // std::cout << "FSP time [ms] : " << latency_ << std::endl;

      // Walking_Pattern_Generator (stack)
      // std::cout << "walking pattern generator" << std::endl;
      start_time_ = std::chrono::system_clock::now();
      walking_pattern_ptr_ = wpg_->walking_pattern_generator(foot_step_ptr_);
      end_time_ = std::chrono::system_clock::now();
      latency_wpg_offline_ = double(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count()) / 1000;
      // std::cout << "WPG time [ms] : " << latency_ << std::endl;
    }

    wall_timer_ = this->create_wall_timer(10ms, std::bind(&RobotManager::Step_Offline, this));
  }

  RobotManager::~RobotManager() {
    std::cout << "finish" << std::endl;
    std::string num = "Num";

    if(DEBUG_MODE_ == true) {
      std::ofstream file_fsp;
      std::string file_fsp_path = "src/Latency_Logs/"+num+"_latency_record_fsp.dat";
      file_fsp.open(file_fsp_path, std::ios::out);
      file_fsp << "unit: [ms]" << std::endl;
      file_fsp << "latency_offline" << std::endl << latency_fsp_offline_ << std::endl;
      file_fsp.close();

      std::ofstream file_wpg;
      std::string file_wpg_path = "src/Latency_Logs/"+num+"_latency_record_wpg.dat";
      file_wpg.open(file_wpg_path, std::ios::out);
      file_wpg << "unit: [ms]" << std::endl;
      file_wpg << "latency_offline" << std::endl << latency_wpg_offline_ << std::endl;
      file_wpg.close();

      std::ofstream file_wsc;
      std::string file_wsc_path = "src/Latency_Logs/"+num+"_latency_record_wsc.dat";
      file_wsc.open(file_wsc_path, std::ios::out);
      latency_wsc_max_ = *std::max_element(all_latency_wsc_.begin(), all_latency_wsc_.end());
      latency_wsc_min_ = *std::min_element(all_latency_wsc_.begin(), all_latency_wsc_.end());
      double sum = 0;
      for(double val : all_latency_wsc_) {
        sum += val;
      }
      latency_wsc_ave_ = sum / all_latency_wsc_.size();
      file_wsc << "unit: [ms]" << std::endl;
      file_wsc << "latency_max" << std::endl << latency_wsc_max_ << std::endl;
      file_wsc << "latency_min" << std::endl << latency_wsc_min_ << std::endl;
      file_wsc << "latency_ave" << std::endl << latency_wsc_ave_ << std::endl;
      file_wsc << "all_latency" << std::endl;
      for(double op : all_latency_wsc_) {
        file_wsc << op << std::endl;
      }
      file_wsc.close();

      std::ofstream file_ctjs;
      std::string file_ctjs_path = "src/Latency_Logs/"+num+"_latency_record_ctjs.dat";
      file_ctjs.open(file_ctjs_path, std::ios::out);
      latency_ctjs_max_ = *std::max_element(all_latency_ctjs_.begin(), all_latency_ctjs_.end());
      latency_ctjs_min_ = *std::min_element(all_latency_ctjs_.begin(), all_latency_ctjs_.end());
      sum = 0;
      for(double val : all_latency_ctjs_) {
        sum += val;
      }
      latency_ctjs_ave_ = sum / all_latency_ctjs_.size();
      file_ctjs << "unit: [ms]" << std::endl;
      file_ctjs << "latency_max" << std::endl << latency_ctjs_max_ << std::endl;
      file_ctjs << "latency_min" << std::endl << latency_ctjs_min_ << std::endl;
      file_ctjs << "latency_ave" << std::endl << latency_ctjs_ave_ << std::endl;
      file_ctjs << "all_latency" << std::endl;
      for(double op : all_latency_ctjs_) {
        file_ctjs << op << std::endl;
      }
      file_ctjs.close();

      std::ofstream file_js;
      std::string file_js_path = "src/Latency_Logs/"+num+"_latency_record_js.dat";
      file_js.open(file_js_path, std::ios::out);
      latency_pub_joint_states_max_ = *std::max_element(all_latency_pub_joint_states_.begin(), all_latency_pub_joint_states_.end());
      latency_pub_joint_states_min_ = *std::min_element(all_latency_pub_joint_states_.begin(), all_latency_pub_joint_states_.end());
      sum = 0;
      for(double val : all_latency_pub_joint_states_) {
        sum += val;
      }
      latency_pub_joint_states_ave_ = sum / all_latency_pub_joint_states_.size();
      file_js << "unit: [ms]" << std::endl;
      file_js << "latency_max" << std::endl << latency_pub_joint_states_max_ << std::endl;
      file_js << "latency_min" << std::endl << latency_pub_joint_states_min_ << std::endl;
      file_js << "latency_ave" << std::endl << latency_pub_joint_states_ave_ << std::endl;
      file_js << "all_latency" << std::endl;
      for(double op : all_latency_pub_joint_states_) {
        file_js << op << std::endl;
      }
      file_js.close();
    }
  }

}