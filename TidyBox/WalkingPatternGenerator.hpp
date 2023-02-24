#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "/*PackageName*//msg/ToWalkingStabilizationController_msgs.msg"
// #include "/*PackageName*//srv/ToKinematics_msgs.msg"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    private:
      rclcpp::Publisher</*PackageName*/::msg::ToWalkingStabilizationController_msgs>::SharedPtr toWSC_pub_ptr;
      rclcpp::Client</*PackageName*/::srv::ToKinematics_msgs>::SharedPtr toKine_clnt_ptr;

      void callback_res(const rclcpp::Client</*PackageName*/::srv::ToKinematics_msgs>::SharedFuture future);
  };
}