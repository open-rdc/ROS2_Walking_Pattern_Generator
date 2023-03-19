#include "rclcpp/rclcpp.hpp"
// #include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    private:
      // rclcpp::Publisher<msgs_package::msg::ToWalkingStabilizationControllerMessage>::SharedPtr toWSC_pub_ptr;
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_FK_clnt;
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_IK_clnt;

      void callback_res(const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future);
  };
}