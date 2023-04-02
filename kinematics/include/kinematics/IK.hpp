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

      double sign(double arg = 0);  // return 1 or -1 (argが>=0なら1, <0なら-1を返す)

      Eigen::Vector3d Array2Vector(std::array<double, 3> array);  // std::array型をEigen::Vector3d型に変換（３次元）
      Eigen::Matrix3d Array2Matrix(std::array<double, 9> array);  // std::array型をEigen::Matrix3d型に変換（3*3行列）

      std::array<double, 6> IK(
        std::array<Eigen::Vector3d, 7> P_leg,
        Eigen::Vector3d P_target_leg,
        Eigen::Matrix3d R_target_leg
      );

 
      rclcpp::Service<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_srv_;

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

      // std::array<double, 6> IK_resultR_;  // 未使用
      // std::array<double, 6> IK_resultL_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}