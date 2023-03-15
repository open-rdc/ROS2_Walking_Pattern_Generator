/* MEMO
ヘッダファイルは、Kinematicsで１つにまとめるのがBetterだと思う。ひとまず、別々で作成している。
ヘッダファイルに限らず、複数serviceを1nodeでspinさせたりも考えられる。汚くなりそうだけども。
あと、汎用的な関数（FK、IK、回転行列）は他cppとして作成して、ライブラリ化する方法は？
*/

#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace kinematics
{
  class IKSrv : public rclcpp::Node {
    public:
      IKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      void IK_SrvServer(
        const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
        std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
      );

      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      std::array<double, 6> IK(
        std::array<Eigen::Matrix3d, 6> R_leg,
        std::array<double, 6> Q_leg,
        Eigen::Vector3d P_target_leg,
        Eigen::Matrix3d R_target_leg
      );

      double sign(double arg = 0);

      rclcpp::Service<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_srv_ptr;

      const float pi = 3.141593;  // 四捨五入済み

      std::array<Eigen::Matrix3d, 6> R_legR;
      std::array<Eigen::Vector3d, 7> P_legR;
      std::array<Eigen::Matrix3d, 6> R_legL;
      std::array<Eigen::Vector3d, 7> P_legL;
      std::array<double, 6> Q_legR;
      std::array<double, 6> Q_legL;

      Eigen::Vector3d P_target_legR;
      Eigen::Matrix3d R_target_legR;
      Eigen::Vector3d P_target_legL;
      Eigen::Matrix3d R_target_legL;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}