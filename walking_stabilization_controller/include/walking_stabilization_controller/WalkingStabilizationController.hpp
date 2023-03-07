#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "msgs_package/srv/to_webots_robot_handler_message.hpp"

namespace walking_stabilization_controller
{
  class WalkingStabilizationController : public rclcpp::Node {
    public:
      WalkingStabilizationController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    private:
      rclcpp::Subscription<msgs_package::msg::ToWalkingStabilizationControllerMessage>::SharedPtr toWSC_sub_ptr;
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_clnt_ptr;
      rclcpp::Service<msgs_package::srv::ToWebotsRobotHandlerMessage>::SharedPtr toWRH_srv_ptr;

      void callback_sub(const msgs_package::msg::ToWalkingStabilizationControllerMessage::SharedPtr sub_data);
      void callback_res(const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future);
      void WSC_SrvServer(
        const std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Request> request,
        std::shared_ptr<msgs_package::srv::ToWebotsRobotHandlerMessage::Response> response
      );
  };
}