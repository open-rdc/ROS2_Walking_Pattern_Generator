#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Msgs_Package/msg/ToWalkingStabilizationController_msgs.msg"
#include "Msgs_Package/srv/ToKinematics_msgs.msg"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    private:
      rclcpp::Publisher<Msgs_Package::msg::ToWalkingStabilizationController_msgs>::SharedPtr toWSC_pub_ptr;
      rclcpp::Client<Msgs_Package::srv::ToKinematics_msgs>::SharedPtr toKine_clnt_ptr;

      void callback_res(const rclcpp::Client<Msgs_Package::srv::ToKinematics_msgs>::SharedFuture future);
  };
}