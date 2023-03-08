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

namespace Kinematics
{
  class FKSrv : public rclcpp::Node {
    public:
      FKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      rclcpp::Service<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_srv_ptr;

      void FK_SrvServer(
        const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
        std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
      );

      const float pi = 3.141593;  // 四捨五入済み

      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      Eigen::Matrix3d R_target;
      std::array<Eigen::Matrix3d, 6> R_leg;
      Eigen::Vector3d P_target;
      std::array<Eigen::Vector3d, 7> P_legR;
      std::array<Eigen::Vector3d, 7> P_legL;
      std::array<double, 6> rad_leg;
  };
}