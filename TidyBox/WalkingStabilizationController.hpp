#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "Msgs_Package/msg/ToWalkingStabilizationController_msgs.msg"
// #include "Msgs_Package/srv/ToKinematics_msgs.msg"
// #include "Msgs_Package/srv/ToWebotsRobotHandler_msgs.msg"

namespace walking_stabilization_controller
{
  class WalkingStabilizationController : public rclcpp::Node {
    public:
      WalkingStabilizationController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    private:
      rclcpp::Subscription<Msgs_Package::msg::ToWalkingStabilizationController_msgs>::SharedPtr toWSC_sub_ptr;
      rclcpp::Client<Msgs_Package::srv::ToKinematics_msgs>::SharedPtr toKine_clnt_ptr;
      rclcpp::Service<Msgs_Package::srv::ToWebotsRobotHandler_msgs>::SharedPtr toWRH_srv_ptr;

      void callback_sub(const Msgs_Package::msg::ToWalkingPatternGenerator_msgs::SharedPtr sub_data);
      void callback_res(const rclcpp::Client<Msgs_Package::srv::ToKinematics_msgs>::SharedFuture future);
      void WSC_SrvServer(
        const std::shared_ptr<Msgs_Package::srv::ToWebotsRobotHandler_msgs::Request> request,
        std::shared_ptr<Msgs_Package::srv::ToWebotsRobotHandler_msgs::Response> response
      );
  };
}