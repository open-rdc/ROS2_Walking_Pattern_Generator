#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

namespace walking_stabilization_controller
{
  class WalkingStabilizationController : public rclcpp::Node {
    public:
      WalkingStabilizationController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      rclcpp::Subscription<msgs_package::msg::ToWalkingStabilizationControllerMessage>::SharedPtr toWSC_sub_;
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_FK_clnt_;
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_IK_clnt_;
      rclcpp::Service<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedPtr toWRH_srv_;

      // to walking stabilization controller message. pub: WPG <=> sub: WSC
      Eigen::Vector3d P_target_legR_;
      Eigen::Vector3d P_target_legL_;
      std::array<double, 6> Q_target_legR_;
      std::array<double, 6> Q_target_legL_;
      std::array<double, 6> dQ_target_legR_;
      std::array<double, 6> dQ_target_legL_;

      // to kinematics message. req: WSC <=> res: FK or IK
      Eigen::Vector3d P_result_legR_;
      Eigen::Vector3d P_result_legL_;
      std::array<double, 6> Q_result_legR_;
      std::array<double, 6> Q_result_legL_;

      // to webots robot handler message. req: WRH <=> WSC
      std::array<double, 6> Q_fix_legR_;
      std::array<double, 6> Q_fix_legL_;
      std::array<double, 6> dQ_fix_legR_;
      std::array<double, 6> dQ_fix_legL_;
 
      void callback_sub(const msgs_package::msg::ToWalkingStabilizationControllerMessage::SharedPtr sub_data);
      void callback_res(const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future);
      void WSC_SrvServer(
        const std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Request> request,
        std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Response> response
      );
  };
}