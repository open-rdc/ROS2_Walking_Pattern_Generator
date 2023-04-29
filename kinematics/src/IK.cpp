#include "rclcpp/rclcpp.hpp"
#include "kinematics/IK.hpp"

namespace kinematics
{

  IK::IK(
    const rclcpp::NodeOptions& options
  ) : Node("IK", options) {

  }

}