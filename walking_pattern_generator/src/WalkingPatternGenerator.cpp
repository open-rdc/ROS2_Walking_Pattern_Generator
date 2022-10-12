#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace walking_pattern_generator {
    void WalkingPatternGenerator::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {
        // ROS2の初期化方法がわからないため、強引にコンパイルを通過した方法。
        //int argc = 0;
        //char const * const argv[] = {""};
        //rclcpp::init(argc, argv);
        
        // webots_ros2_driverのsensor周りを参考にすると、以下のような記述。恐らくこの記述も、ROS2の初期化となっているはず
        PluginInterface::init(node, parameters);  // no err. 恐らくコレで良い
        
        auto ros2_node = rclcpp::Node::make_shared("robot_controller");

        // auto robot = node->robot();
        // auto hoge = robot->getMotor("AnkleL");
        // auto hige = hoge->setPosition(10);
        // RCLCPP_INFO(ros2_node->get_logger(), "Hello my mine...");
    }

    void WalkingPatternGenerator::step() {
        while(true);
        // std::cout << "Test, test, test..." << std::endl;
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WalkingPatternGenerator, webots_ros2_driver::PluginInterface
)
