#include "rclcpp/rclcpp.hpp"
#include "robot_manager/robot_manager.hpp"
#include "msgs_package/srv/to_robot_manager.hpp"
#include "msgs_package/srv/to_walking_pattern_generator.hpp"
#include "msgs_package/srv/to_walking_stabilization_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robot_manager {
  RobotManager::RobotManager(
    const rclcpp::NodeOptions &options
  ) : Node("RobotManager", options) {
    
  }
}