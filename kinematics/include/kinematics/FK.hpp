#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace kinematics
{
  class FKSrv : public rclcpp::Node {
    public:
      FKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      void FK_SrvServer(
        const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
        std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
      );

      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      Eigen::Vector3d FK(
        std::array<Eigen::Matrix3d, 6> R_leg,
        std::array<Eigen::Vector3d, 7> P_leg
      );

      rclcpp::Service<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_srv_;

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