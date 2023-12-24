#include "walking_stabilization_controller/WalkingStabilizationController.hpp"

namespace walking_stabilization_controller 
{
  std::unique_ptr<control_plugin_base::WalkingStabilization> Default_WalkingStabilizationController::walking_stabilization_controller(
    const std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr
  ) {
    auto walking_stabilization_ptr = std::make_unique<control_plugin_base::WalkingStabilization>();
    walking_stabilization_ptr->cog_pos_fix = walking_pattern_ptr->cc_cog_pos_ref;
    walking_stabilization_ptr->cog_vel_fix = walking_pattern_ptr->cc_cog_vel_ref;
    walking_stabilization_ptr->zmp_pos_fix = walking_pattern_ptr->wc_foot_land_pos_ref;

    // std::cout << "Here is default walking stabilization controller class." << std::endl;

    return walking_stabilization_ptr;
  }

  Default_WalkingStabilizationController::Default_WalkingStabilizationController() {
    node_ptr_ = rclcpp::Node::make_shared("WalkingStabilizationController");
    client_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "RobotParameterServer");

    RCLCPP_INFO(node_ptr_->get_logger(), "Start Up WalkingStabilizationController.");
  }
}
  // std::unique_ptr<control_plugin_base::WalkingStabilization> Default_WalkingStabilizationController::walking_stabilization_controller(
  //   const std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr, 
  //   const uint32_t control_step
  // ) {
  //   auto walking_stabilization_ptr = std::make_unique<control_plugin_base::WalkingStabilization>();

  //   if(control_step > walking_pattern_ptr->cog_pos_ref.size()-1) {
  //     max_step = walking_pattern_ptr->cog_pos_ref.size()-1;
  //     walking_stabilization_ptr->cog_pos_fix = walking_pattern_ptr->cog_pos_ref.at(max_step);
  //     walking_stabilization_ptr->cog_vel_fix = walking_pattern_ptr->cog_vel_ref.at(max_step);
  //     walking_stabilization_ptr->zmp_pos_fix = walking_pattern_ptr->foot_pos_ref.at(max_step);
  //   }

  //   walking_stabilization_ptr->cog_pos_fix = walking_pattern_ptr->cog_pos_ref.at(control_step);
  //   walking_stabilization_ptr->cog_vel_fix = walking_pattern_ptr->cog_vel_ref.at(control_step);
  //   walking_stabilization_ptr->zmp_pos_fix = walking_pattern_ptr->foot_pos_ref.at(control_step);

  //   // std::cout << "Here is default walking stabilization controller class." << std::endl;

  //   return walking_stabilization_ptr;
  // }


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(walking_stabilization_controller::Default_WalkingStabilizationController, control_plugin_base::WalkingStabilizationController)