#include "rclcpp/rclcpp.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace old_kinematics
{
  class FK : public rclcpp::Node {
    public:
      FK(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      Eigen::Vector3d FK_calc(
        std::array<Eigen::Matrix3d, 6> R_leg,
        std::array<Eigen::Vector3d, 7> P_leg
      );

      const float pi = 3.141593;  // 四捨五入済み

      std::array<Eigen::Matrix3d, 6> R_legR_;
      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Matrix3d, 6> R_legL_;
      std::array<Eigen::Vector3d, 7> P_legL_;
      std::array<double, 6> Q_legR_;
      std::array<double, 6> Q_legL_;

      Eigen::Vector3d FK_resultR_;
      Eigen::Vector3d FK_resultL_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}