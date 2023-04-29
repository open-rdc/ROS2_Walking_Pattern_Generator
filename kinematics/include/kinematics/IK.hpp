#include "rclcpp/rclcpp.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace kinematics
{
  class IK : public rclcpp::Node {
    public:
      IK(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      double sign(double arg = 0);  // return 1 or -1 (argが>=0なら1, <0なら-1を返す)

      Eigen::Vector3d Array2Vector(std::array<double, 3> array);  // std::array型をEigen::Vector3d型に変換（３次元）
      Eigen::Matrix3d Array2Matrix(std::array<double, 9> array);  // std::array型をEigen::Matrix3d型に変換（3*3行列）

      std::array<double, 6> IK_calc(
        std::array<Eigen::Vector3d, 7> P_leg,
        Eigen::Vector3d P_target_leg,
        Eigen::Matrix3d R_target_leg
      );

      const float pi_ = 3.141593;  // 四捨五入済み

      std::array<Eigen::Matrix3d, 6> R_legR_;
      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Matrix3d, 6> R_legL_;
      std::array<Eigen::Vector3d, 7> P_legL_;
      std::array<double, 6> Q_legR_;
      std::array<double, 6> Q_legL_;

      Eigen::Vector3d P_target_legR_;
      Eigen::Matrix3d R_target_legR_;
      Eigen::Vector3d P_target_legL_;
      Eigen::Matrix3d R_target_legL_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}