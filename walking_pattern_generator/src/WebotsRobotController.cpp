#include "walking_pattern_generator/WebotsRobotController.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace walking_pattern_generator {
    void WebotsRobotController::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {
        // ROS2の初期化方法がわからないため、強引にコンパイルを通過した方法。
        //int argc = 0;
        //char const * const argv[] = {""};
        //rclcpp::init(argc, argv);
        
        // webots_ros2_driverのsensor周りを参考にすると、以下のような記述。恐らくこの記述も、ROS2の初期化となっているはず。（未検証。未コンパイル）
        // PluginInterface::init(node, parameters);
        
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
