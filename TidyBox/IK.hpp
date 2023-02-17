/* MEMO
ヘッダファイルは、Kinematicsで１つにまとめるのがBetterだと思う。ひとまず、別々で作成している。
ヘッダファイルに限らず、複数serviceを1nodeでspinさせたりも考えられる。汚くなりそうだけども。
あと、汎用的な関数（FK、IK、回転行列）は他cppとして作成して、ライブラリ化する方法は？
*/

#include "rclcpp/rclcpp.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace Kinematics
{
  class IKSrv : public rclcpp::Node {
    public:
      IKSrv(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    private:
      rclcpp::Service</*SrvMsg*/>::SharedPtr srv_ptr;

      void IK_SrvServer(
        const std::shared_ptr</*自身で定義したSrvMsg*/::Request> request,
        std::shared_ptr</*上に同じ*/::Response> response
      );

      const float pi = 3.141593;  // 四捨五入済み

      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      double sign(double arg = 0);

      Eigen::Matrix3d R_target_ptr;
      std::array<Eigen::Matrix3d, 6> R;
      Eigen::Vector3d P_target;
      std::array<Eigen::Vector3d, 7> P;
      std::array<float, 6> rad;
  };
}