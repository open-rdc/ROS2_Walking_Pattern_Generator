#include "foot_step_planner/FootStepPlanner.hpp"

namespace foot_step_planner
{
  std::unique_ptr<control_plugin_base::FootStep> Default_FootStepPlanner::foot_step_planner(void) {
    auto foot_step_ptr = std::make_unique<control_plugin_base::FootStep>();
    
    foot_step_ptr->walking_step_time = WALKING_CYCLE_;  // 歩行周期[s]
    foot_step_ptr->foot_pos = {  // 着地位置{x, y}[m]
      {0.0, 0.037},
      {0.0, 0.074},
      {0.03, 0.0},
      {0.06, 0.074},
      {0.09, 0.0},
      {0.12, 0.074},
      {0.12, 0.037},
      {0.12, 0.037}
    };
    foot_step_ptr->waist_height = WAIST_HEIGHT_;
    //foot_step_ptr->waist_height = 171.856 / 1000;  // 腰高さ

    // std::cout << "Here is default foot_step_controller plugin." << std::endl;

    return foot_step_ptr;
  }

  Default_FootStepPlanner::Default_FootStepPlanner() {
    node_ptr_ = rclcpp::Node::make_shared("FootStepPlanner");
    client_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "RobotParameterServer");

    WALKING_CYCLE_ = client_param_->get_parameter<double>("control_times.walking_cycle");
    WAIST_HEIGHT_ = client_param_->get_parameter<double>("control_constant.waist_pos_z");

    RCLCPP_INFO(node_ptr_->get_logger(), "Start Up FootStepPlanner.");
  }
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(foot_step_planner::Default_FootStepPlanner, control_plugin_base::FootStepPlanner)