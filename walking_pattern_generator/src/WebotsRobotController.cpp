#include "walking_pattern_generator/WebotsRobotController.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace walking_pattern_generator {
    void WebotsRobotController::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {
        int argc = 0;
        char const * const argv[] = {""};
        rclcpp::init(argc, argv);
        auto ros2_node = rclcpp::Node::make_shared("robot_controller");

        auto robot = node->robot();
        RCLCPP_INFO(ros2_node->get_logger(), "Hello my mine...");
    }

    void WebotsRobotController::step() {
        std::cout << "Test, test, test..." << std::endl;
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WebotsRobotController, webots_ros2_driver::PluginInterface
)