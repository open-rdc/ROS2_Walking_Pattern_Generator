#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_Jacobian.hpp"

#include "Eigen/Dense"

namespace kinematics
{
  class Default_Jacobian : public rclcpp::Node {
    public:
      Jacobian(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

      Eigen::Matrix<double, 6, 6> JacobiMatrix_leg(
        std::array<double, 6> Q_leg,
        std::array<Eigen::Vector3d, 6> UnitVec_leg,
        std::array<Eigen::Vector3d, 7> P_leg
      );

    private:
      kinematics::FK FK_;

  };
}

#endif