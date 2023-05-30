#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/stabilization_control.hpp"
#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"
#include "walking_stabilization_controller/visibility_control.h"

#include "Eigen/Dense"

namespace robot_manager
{
  class WalkingStabilizationController : public rclcpp::Node {
    public:
      WALKING_STABILIZATION_CONTROLLER_PUBLIC
      WalkingStabilizationController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      void WSC_Server(
        const std::shared_ptr<msgs_package::srv::StabilizationControl::Request> request,
        std::shared_ptr<msgs_package::srv::StabilizationControl::Response> response
      );

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/

      // 共有ライブラリの実体化
      kinematics::FK FK_;
      kinematics::IK IK_;

      rclcpp::Service<msgs_package::srv::StabilizationControl>::SharedPtr srv_stabilization_control_;

      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_;

    rclcpp::CallbackGroup::SharedPtr cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  };
}