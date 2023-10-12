#include "kinematics/Jacobian.hpp"

using namespace Eigen;

namespace kinematics
{
  void Default_Jacobian::jacobian(
    const std::shared_ptr<control_plugin_base::LegStates_ToJac> leg_states_jac_ptr,
    Eigen::Matrix<double, 6, 6>& leg_jacobian_ptr
  ) {
    leg_jacobian_ptr = MatrixXd::Zero(6, leg_states_jac_ptr->unit_vec.max_size());

    std::array<Eigen::Vector3d, 6> P_FK_leg;
    Vector3d end_eff_pos = {0, 0, 0};
    std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_fk_ptr = std::make_shared<control_plugin_base::LegStates_ToFK>();
    leg_states_fk_ptr->joint_ang = leg_states_jac_ptr->joint_ang;
    leg_states_fk_ptr->link_len = leg_states_jac_ptr->link_len;
    for(int joint_point = 0; joint_point < int(leg_states_jac_ptr->unit_vec.max_size()); joint_point++) {
      fk_->forward_kinematics(leg_states_fk_ptr, joint_point, end_eff_pos);
      P_FK_leg[joint_point] = end_eff_pos;
      // std::cout << P_FK_leg[joint_point].transpose() << std::endl;
    }

    Vector3d mat_leg = Vector3d::Zero(3);
    Vector3d pt_P_leg = Vector3d::Zero(3);
    for(int tag = 0; tag < int(leg_states_jac_ptr->unit_vec.max_size()); tag++) {
      if(tag == int(leg_states_jac_ptr->unit_vec.max_size()-1)) {
        mat_leg = Vector3d::Zero(3);
      }
      else { 
        pt_P_leg = P_FK_leg[int(leg_states_jac_ptr->unit_vec.max_size())-1] - P_FK_leg[tag];
        // std::cout << "pt_P_leg: " << pt_P_leg.transpose() << ", " << P_FK_leg[int(leg_states_jac_ptr->unit_vec.max_size())-1].transpose() << ", " << P_FK_leg[tag].transpose() << std::endl;
        mat_leg = leg_states_jac_ptr->unit_vec[tag].cross(pt_P_leg);
      }

      for(int i = 0; i < 3; i++) {
        leg_jacobian_ptr(i, tag) = mat_leg[i];
        leg_jacobian_ptr(i+3, tag) = leg_states_jac_ptr->unit_vec[tag][i];
      }
    }
  }

  Default_Jacobian::Default_Jacobian() {
    // std::cout << "Init Jac" << std::endl;
    fk_ = fk_loader.createSharedInstance("kinematics::Default_ForwardKinematics");
  }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(kinematics::Default_Jacobian, control_plugin_base::Jacobian)