#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"

#include "Eigen/Dense"
using namespace Eigen;

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<control_plugin_base::WalkingPatternGenerator> wpg_loader("robot_manager", "control_plugin_base::WalkingPatternGenerator");
  pluginlib::ClassLoader<control_plugin_base::FootStepPlanner> fsp_loader("robot_manager", "control_plugin_base::FootStepPlanner");
  pluginlib::ClassLoader<control_plugin_base::WalkingStabilizationController> wsc_loader("robot_manager", "control_plugin_base::WalkingStabilizationController");
  pluginlib::ClassLoader<control_plugin_base::ConvertToJointStates> ctjs_loader("robot_manager", "control_plugin_base::ConvertToJointStates");
  pluginlib::ClassLoader<control_plugin_base::ForwardKinematics> fk_loader("robot_manager", "control_plugin_base::ForwardKinematics");

  try
  {
    std::shared_ptr<control_plugin_base::WalkingPatternGenerator> wpg = wpg_loader.createSharedInstance("walking_pattern_generator::LinearInvertedPendulumModel");
    std::shared_ptr<control_plugin_base::FootStepPlanner> fsp = fsp_loader.createSharedInstance("foot_step_planner::Default_FootStepPlanner");
    std::shared_ptr<control_plugin_base::WalkingStabilizationController> wsc = wsc_loader.createSharedInstance("walking_stabilization_controller::Default_WalkingStabilizationController");
    std::shared_ptr<control_plugin_base::ConvertToJointStates> ctjs = ctjs_loader.createSharedInstance("convert_to_joint_states::Default_ConvertToJointStates");
    std::shared_ptr<control_plugin_base::ForwardKinematics> fk = fk_loader.createSharedInstance("kinematics::Default_ForwardKinematics");

// Foot_Step_Planner
    std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr = fsp->foot_step_planner();

// Walking_Pattern_Generator
    std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = wpg->walking_pattern_generator(foot_step_ptr);

// Walking_Stabilization_Controller
    std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr = wsc->walking_stabilization_controller(walking_pattern_ptr);

// Convert_to_Joint_States
    std::shared_ptr<control_plugin_base::LegJointStatesPattern> leg_joint_states_pat_ptr = ctjs->convert_into_joint_states(walking_stabilization_ptr);

    // DEBUG: FKの動作確認
    std::shared_ptr<control_plugin_base::LegStates> legR_states_ptr = std::make_shared<control_plugin_base::LegStates>();
    legR_states_ptr->joint_ang = {0, 0, 0, 0, 0, 0};  // rad
    legR_states_ptr->link_len = { Vector3d{0,0,1},
                                  Vector3d{0,0,1},
                                  Vector3d{0,0,1},
                                  Vector3d{0,0,1},
                                  Vector3d{0,0,1},
                                  Vector3d{0,0,1},
                                  Vector3d{0,0,1}
    };
    for(int8_t point = 0; point < 7; point++) {
      legR_states_ptr->joint_point = point;
      fk->forward_kinematics(legR_states_ptr);
      std::cout << "[DEBUG]: [FK]: joint_point" << point << ", end_effecter_position: { " << legR_states_ptr->end_eff_pos[0] << ", " << legR_states_ptr->end_eff_pos[1] << ", " << legR_states_ptr->end_eff_pos[2] << " }" << std::endl;
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("ERROR!!: %s\n", ex.what());
  }
  
  return 0;
}