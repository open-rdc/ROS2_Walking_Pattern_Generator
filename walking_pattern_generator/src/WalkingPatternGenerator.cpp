#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

// webots関連の関数を使いやすくするために、using namepaceで省略できるようにしたほうが良いかも。

namespace walking_pattern_generator {
    // rclcpp::Nodeの継承。Node化するために実装。publisherをstep()で実行すればOK。
    Node_WalkingPatternGenerator::Node_WalkingPatternGenerator (
        const std::string &name_space,
        const rclcpp::NodeOptions &options
    ): Node("robot_controller", name_space, options) {
        publisher = rclcpp::Node::create_publisher<std_msgs::msg::String>("test", rclcpp::QoS(10));
    }

    void WalkingPatternGenerator::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {

        /* これはpluginである。そのため、ココでnodeの宣言やROS2とnodeの初期化は行わない。別途、main関数含むプログラムから使う。 */

        // 以下のような処理をココで行えば良い
        Node = node;
        robot = Node->robot();
        motor[0] = robot->getMotor("AnkleL");
        positionSensor[0] = robot->getPositionSensor("AnkleLS");  // value type: double
        accelerometer = robot->getAccelerometer("Accelerometer");  // values type: double* (need check document)
        gyro = robot->getGyro("Gyro");  // values type: double* (need check document)
        positionSensor[0]->webots::PositionSensor::enable(100);  // sampling period: 100[ms]. この宣言の直後に値は得られない。1周期後（ここでは100[ms）後に１つ目の値が得られる。
        accelerometer->webots::Accelerometer::enable(100);  // sampling period: 100[ms]
        gyro->webots::Gyro::enable(100);  // sampling period: 100[ms]

        double hage = positionSensor[0]->webots::PositionSensor::getValue();
        motor[0]->webots::Motor::setPosition(1);  // 単位: [rad]
        RCLCPP_INFO(node->get_logger(), "Hello my mine...%lf", hage);
    }

    void WalkingPatternGenerator::step() {
        double hage;
        std::cout << "Ride On!" << std::endl;
        hage = positionSensor[0]->webots::PositionSensor::getValue();
        RCLCPP_INFO(Node->get_logger(), "hogehoge: %lf", hage);
        sleep(1.0);  // sleepすると、webotsのシミュレーション時間も止まる。止まるというより、時間の進みが*0.08とかになる。サンプリングも止まるから、注意。
        // サンプリング周期を設定して、その周期でstepを進めてやれば、リアルタイム性のある制御となる。
            // 計算時間が確保できないなら、その間シミュレートを止めることもできる。<ーはじめはコレでいきたい。
       
        // auto pub = Node->create_publisher<std::string>("test");
        // ココからロボットのデータを逐次Publishをしたい。
        // 他nodeから計算結果などもsubscribeしたい。
            // 恐らく、hppの方にnode作成用の関数を作成し、実体化させる必要がある。
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WalkingPatternGenerator, webots_ros2_driver::PluginInterface
)
