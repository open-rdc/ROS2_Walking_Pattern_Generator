#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_robot_manager.hpp"
#include "msgs_package/srv/to_walking_pattern_generator.hpp"
#include "msgs_package/srv/to_walking_stabilization_controller.hpp"

namespace robot_manager {
  class RobotManager : public rclcpp::Node {
    public:
      RobotManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      rclcpp::Service<msgs_package::srv::ToRobotManager>::SharedPtr RM_srv_;
      rclcpp::Client<msgs_package::srv::ToWalkingPatternGenerator>::SharedPtr WPG_clnt_;
      rclcpp::Client<msgs_package::srv::ToWalkingStabilizationController>::SharedPtr WSC_clnt_;

      void RM_Server(
        const std::shared_ptr<msgs_package::srv::ToRobotManager::Request> request,
        std::shared_ptr<msgs_package::srv::ToRobotManager::Response> response
      );

      rclcpp::CallbackGroup::SharedPtr callback_group_ = nullptr;

  };
}