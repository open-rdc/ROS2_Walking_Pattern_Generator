#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>

#include <fstream>

namespace logger {

  class RobotFeedbackLogger : public rclcpp::Node {
    public:
      RobotFeedbackLogger(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
      ) : Node("RobotFeedbackLogger", options) {

      }

      ~RobotFeedbackLogger() {

      }

    private:

  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<logger::RobotFeedbackLogger>());
  rclcpp::shutdown();

  return 0;
}