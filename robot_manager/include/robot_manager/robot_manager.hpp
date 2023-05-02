#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_robot_manager.hpp"
#include "msgs_package/srv/to_walking_pattern_generator.hpp"
#include "msgs_package/srv/to_walking_stabilization_controller.hpp"

namespace robot_manager {
  class RobotManager : public rclcpp::Node {
    public:
      RobotManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

  };
}