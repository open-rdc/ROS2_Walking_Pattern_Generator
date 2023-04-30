#include "rclcpp/rclcpp.hpp"
// #include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
// #include "msgs_package/srv/to_kinematics_message.hpp"
// #include "msgs_package/srv/to_webots_robot_handler_message.hpp"
#include "msgs_package/srv/to_walking_stabilization_controller.hpp"

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

      rclcpp::Service<msgs_package::srv::ToWalkingStabilizationController>::SharedPtr WSC_srv_;

      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}