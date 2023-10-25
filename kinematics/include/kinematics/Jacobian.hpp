#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_Jacobian.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"

#include "Eigen/Dense"

namespace kinematics
{
  pluginlib::ClassLoader<control_plugin_base::ForwardKinematics> fk_loader("robot_manager", "control_plugin_base::ForwardKinematics");

  class Default_Jacobian : public control_plugin_base::Jacobian {
    public:
      Default_Jacobian();
      ~Default_Jacobian(){}

      void jacobian(
        const std::shared_ptr<control_plugin_base::LegStates_ToJac> leg_states_jac_ptr,
        Eigen::Matrix<double, 6, 6>& leg_jacobian
      ) override;

    private:
      std::shared_ptr<control_plugin_base::ForwardKinematics> fk_;
  };
}

#endif