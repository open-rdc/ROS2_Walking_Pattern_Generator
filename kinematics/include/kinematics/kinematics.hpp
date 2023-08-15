#ifndef KINEMATICS__KINEMATICS_HPP_
#define KINEMATICS__KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "kinematics/visibility_control.h"

#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

namespace kinematics
{
  class Kinematics : public rclcpp::Node {
    public:
      Kinematics(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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