#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

// webots関連の関数を使いやすくするために、using namepaceで省略できるようにしたほうが良いかも。

namespace walking_pattern_generator 
{
    void WalkingPatternGenerator::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {
        // NODE
        Node = node;
        robot = Node->robot();
        motor[0] = robot->getMotor("AnkleL");
        positionSensor[0] = robot->getPositionSensor("AnkleLS");  // value type: double
        accelerometer = robot->getAccelerometer("Accelerometer");  // values type: double* (need check document)
        gyro = robot->getGyro("Gyro");  // values type: double* (need check document)

        positionSensor[0]->webots::PositionSensor::enable(100);  // sampling period: 100[ms]. この宣言の直後に値は得られない。1周期後（ここでは100[ms）後に１つ目の値が得られる。
        accelerometer->webots::Accelerometer::enable(100);  // sampling period: 100[ms]
        gyro->webots::Gyro::enable(100);  // sampling period: 100[ms]

        RCLCPP_INFO(Node->get_logger(), "Hello my mind...");

        __pub = node->create_publisher<std_msgs::msg::String>("test", rclcpp::QoS(10));
        
    }

    void WalkingPatternGenerator::step() {
        // NODE: CHECK SENSOR DATA
        positionSensorValue[0] = positionSensor[0]->webots::PositionSensor::getValue();  // 毎stepで値を再取得しないと、値が更新されない。
        accelerometerValue = accelerometer->webots::Accelerometer::getValues();  // 毎stepで値を再取得せずとも、値は更新される。値を保持するためには、他変数にコピーする必要がある。
        gyroValue = gyro->webots::Gyro::getValues();  // 加速度センサ値と同様。

        motorValue[0] = 1;
        motor[0]->webots::Motor::setVelocity(0.1);
        motor[0]->webots::Motor::setPosition(motorValue[0]);  // 単位: [rad]
        
        RCLCPP_INFO(Node->get_logger(), "pos: %F, acc: [x: %F, y: %F, z: %F], gyro: [x: %F, y: %F, z: %F] ", 
            positionSensorValue[0], accelerometerValue[0], accelerometerValue[1], accelerometerValue[2],gyroValue[0], gyroValue[1], gyroValue[2]);
        

        // 以上のstep周期: 約 25[ms]
        // データを100[ms]で取得しており、標準出力でセンサ値が４回の出力記録の周期で変化しているため。

        // PUBLISH
        // auto datamsg = std::make_shared<std_msgs::msg::String>();
        // static int count = 0;
        // datamsg->data = "Hello: " + std::to_string(count++);
        // __pub->publish(*datamsg);

        // TEST
        // double hage;
        // std::cout << "Ride On!" << std::endl;
        // hage = positionSensor[0]->webots::PositionSensor::getValue();
        // RCLCPP_INFO(Node->get_logger(), "hogehoge: %lf", hage);
        // sleep(1.0);  
        
        // sleepすると、webotsのシミュレーション時間も止まる。止まるというより、時間の進みが*0.08とかになる。サンプリングも止まるから、注意。
        // サンプリング周期を設定して、その周期でstepを進めてやれば、リアルタイム性のある制御となる。
            // 計算時間が確保できないなら、その間シミュレートを止めることもできる。<ーはじめはコレでいきたい。
        // rosbagも有り

        // ココからロボットのデータを逐次Publishをしたい。
        // 他nodeから計算結果などもsubscribeしたい。
            // 別に新規でpluginを作成し、PubとSubを分けたほうが良さげ？
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WalkingPatternGenerator, webots_ros2_driver::PluginInterface
)
