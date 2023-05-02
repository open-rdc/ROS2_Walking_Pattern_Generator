#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_walking_stabilization_controller.hpp"
#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

namespace walking_stabilization_controller
{
  class WalkingStabilizationController : public rclcpp::Node {
    public:
      WalkingStabilizationController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      void WSC_Server(
        const std::shared_ptr<msgs_package::srv::ToWalkingStabilizationController::Request> request,
        std::shared_ptr<msgs_package::srv::ToWalkingStabilizationController::Response> response
      );

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/

      // 共有ライブラリの実体化
      kinematics::FK FK_;
      kinematics::IK IK_;

      rclcpp::Service<msgs_package::srv::ToWalkingStabilizationController>::SharedPtr WSC_srv_;

      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_;
  };
}