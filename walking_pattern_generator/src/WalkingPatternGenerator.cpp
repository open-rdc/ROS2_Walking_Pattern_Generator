#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

namespace walking_pattern_generator {
    void WalkingPatternGenerator::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {

        /* これはpluginである。そのため、ココでnodeの宣言やROS2とnodeの初期化は行わない。別途、main関数含むプログラムから使う。 */
        /* 各データや各関節名を得る変数の宣言は、hppの方で宣言したほうが良い。さすればinitでもstepでもアクセスできる。privateかprotectedか。 */

        // 以下のような処理をココで行えば良い
        auto robot = node->robot();
        auto hoge = robot->getMotor("AnkleL");
        auto hige = robot->getPositionSensor("AnkleLS");
        double hage = hige->webots::PositionSensor::getValue();
        hoge->webots::Motor::setPosition(1);  // 単位: [rad]
        RCLCPP_INFO(node->get_logger(), "Hello my mine...%lf", hage);
    }

    void WalkingPatternGenerator::step() {
        std::cout << "Ride On!" << std::endl;
        sleep(1.0);  // sleepすると、webotsのシミュレーション時間も止まる。止まるというより、時間の進みが*0.08とかになる。
        // ココからロボットのデータを逐次Publishをしたい。
        // 他nodeから計算結果などもsubscribeしたい。
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WalkingPatternGenerator, webots_ros2_driver::PluginInterface
)
