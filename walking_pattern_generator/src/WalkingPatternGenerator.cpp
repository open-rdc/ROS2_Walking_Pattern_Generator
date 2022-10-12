#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace walking_pattern_generator {
    void WalkingPatternGenerator::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {

        /* これはpluginである。そのため、ココでnodeの宣言やROS2とnodeの初期化は行わない。別途、main関数含むプログラムから使う。 */

        // 以下のような処理をココで行えば良い
        // auto robot = node->robot();
        // auto hoge = robot->getMotor("AnkleL");
        // auto hige = hoge->setPosition(10);
        RCLCPP_INFO(node->get_logger(), "Hello my mine...");
    }

    void WalkingPatternGenerator::step() {
        std::cout << "Ride On!" << std::endl;
        // ココからロボットのデータを逐次Publishをしたい。
        // 他nodeから計算結果などもsubscribeしたい。
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WalkingPatternGenerator, webots_ros2_driver::PluginInterface
)
