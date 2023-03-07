/* MEMO
ヘッダファイルは、Kinematicsで１つにまとめるのがBetterだと思う。ひとまず、別々で作成している。
ヘッダファイルに限らず、複数serviceを1nodeでspinさせたりも考えられる。汚くなりそうだけども。
あと、汎用的な関数（FK、IK、回転行列）は他cppとして作成して、ライブラリ化する方法は？
*/

#include "rclcpp/rclcpp.hpp"
#include "Msgs_Package/srv/ToKinematics_msgs.msg"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace Kinematics
{
  class FKSrv : public rclcpp::Node {
    public:
      FKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      rclcpp::Service<Msgs_Package::srv::ToKinematics_msgs>::SharedPtr toKine_srv_ptr;

      void FK_SrvServer(
        const std::shared_ptr<Msgs_Package::srv::ToKinematics_msgs::Request> request,
        std::shared_ptr<Msgs_Package::srv::ToKinematics_msgs::Response> response
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
      std::array<float, 6> rad_leg;
  };
}