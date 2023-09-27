#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "robot_manager/control_plugin_bases/PluginBase_FootStepPlanner.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ConvertToJointStates.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_InverseKinematics.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_Jacobian.hpp"

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
  pluginlib::ClassLoader<control_plugin_base::InverseKinematics> ik_loader("robot_manager", "control_plugin_base::InverseKinematics");
  pluginlib::ClassLoader<control_plugin_base::Jacobian> jac_loader("robot_manager", "control_plugin_base::Jacobian");

  try
  {
    std::shared_ptr<control_plugin_base::WalkingPatternGenerator> wpg = wpg_loader.createSharedInstance("walking_pattern_generator::LinearInvertedPendulumModel");
    std::shared_ptr<control_plugin_base::FootStepPlanner> fsp = fsp_loader.createSharedInstance("foot_step_planner::Default_FootStepPlanner");
    std::shared_ptr<control_plugin_base::WalkingStabilizationController> wsc = wsc_loader.createSharedInstance("walking_stabilization_controller::Default_WalkingStabilizationController");
    std::shared_ptr<control_plugin_base::ConvertToJointStates> ctjs = ctjs_loader.createSharedInstance("convert_to_joint_states::Default_ConvertToJointStates");
    std::shared_ptr<control_plugin_base::ForwardKinematics> fk = fk_loader.createSharedInstance("kinematics::Default_ForwardKinematics");
    std::shared_ptr<control_plugin_base::InverseKinematics> ik = ik_loader.createSharedInstance("kinematics::Default_InverseKinematics");
    std::shared_ptr<control_plugin_base::Jacobian> jac = jac_loader.createSharedInstance("kinematics::Default_Jacobian");

// Foot_Step_Planner
    std::shared_ptr<control_plugin_base::FootStep> foot_step_ptr = fsp->foot_step_planner();

// Walking_Pattern_Generator
    std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr = wpg->walking_pattern_generator(foot_step_ptr);

// Walking_Stabilization_Controller
    std::shared_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_ptr = wsc->walking_stabilization_controller(walking_pattern_ptr);

// Convert_to_Joint_States
    std::shared_ptr<control_plugin_base::LegJointStatesPattern> leg_joint_states_pat_ptr = ctjs->convert_into_joint_states(walking_stabilization_ptr);

    // DEBUG: IK&FKの動作確認
    std::shared_ptr<control_plugin_base::LegStates_IK> legR_states_IK_ptr = std::make_shared<control_plugin_base::LegStates_IK>();  // 変数名、FKと間違えやすい。
    legR_states_IK_ptr->end_eff_pos = {-0.000, -0.1, -0.063};
    legR_states_IK_ptr->end_eff_rot = Matrix3d{{1, 0, 0},
                                            {0, 1, 0},
                                            {0, 0, 1}
    };
    legR_states_IK_ptr->link_len = {Vector3d{-0.005,-0.037,0},
                                    Vector3d{0,0,0},
                                    Vector3d{0,0,0},
                                    Vector3d{0,0,-0.093},
                                    Vector3d{0,0,-0.093},
                                    Vector3d{0,0,0},
                                    Vector3d{0,0,0}
    };
    ik->inverse_kinematics(legR_states_IK_ptr);
    std::cout << "[DEBUG]: [FK]: end_pos_{" << legR_states_IK_ptr->end_eff_pos.transpose() << "}, joint_ang_{"  << legR_states_IK_ptr->joint_ang[0] << ", "
                                                                                                                << legR_states_IK_ptr->joint_ang[1] << ", "
                                                                                                                << legR_states_IK_ptr->joint_ang[2] << ", "
                                                                                                                << legR_states_IK_ptr->joint_ang[3] << ", "
                                                                                                                << legR_states_IK_ptr->joint_ang[4] << ", "
                                                                                                                << legR_states_IK_ptr->joint_ang[5] << "} "<< std::endl; 

    std::shared_ptr<control_plugin_base::LegStates_FK> legR_states_FK_ptr = std::make_shared<control_plugin_base::LegStates_FK>();
    legR_states_FK_ptr->joint_ang = legR_states_IK_ptr->joint_ang;  // rad
    legR_states_FK_ptr->link_len = legR_states_IK_ptr->link_len;
    legR_states_FK_ptr->joint_point = 6;
    fk->forward_kinematics(legR_states_FK_ptr);
    std::cout << "[DEBUG]: [FK]: joint_point " << 6 << ", end_effecter_position: { "<< legR_states_FK_ptr->end_eff_pos[0]
                                                                            << ", " << legR_states_FK_ptr->end_eff_pos[1] 
                                                                            << ", " << legR_states_FK_ptr->end_eff_pos[2] << " }" << std::endl;

    // DEBUG: Jacobianの動作確認
    std::shared_ptr<control_plugin_base::LegStates_ToJac> legR_states_jac_ptr = std::make_shared<control_plugin_base::LegStates_ToJac>();
    legR_states_jac_ptr->joint_ang = legR_states_FK_ptr->joint_ang;
    legR_states_jac_ptr->link_len = legR_states_IK_ptr->link_len;
    legR_states_jac_ptr->unit_vec = {  // legR joint unit vector
      Vector3d(0, 0, 1),
      Vector3d(1, 0, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(0, 1, 0),
      Vector3d(1, 0, 0)
    };
    Eigen::Matrix<double, 6, 6> legR_jacobian_ptr;
    jac->jacobian(legR_states_jac_ptr, legR_jacobian_ptr);  /// (引数, 返り値)
    std::cout << legR_jacobian_ptr << std::endl;
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("ERROR!!: %s\n", ex.what());
  }
  
  return 0;
}