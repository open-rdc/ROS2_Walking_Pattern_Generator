#ifndef WALKING_PATTERN_GENERATOR_HPP
#define WALKING_PATTERN_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace walking_pattern_generator 
{
    class WalkingPatternGenerator : public webots_ros2_driver::PluginInterface {
        public:
            void init(
                webots_ros2_driver::WebotsNode *node, 
                std::unordered_map<std::string, std::string> &parameters
                ) override;
            void step() override;

        private:
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr __pub;  // publisher
            webots_ros2_driver::WebotsNode *Node;  // node
            webots::Robot *robot;  // robot
            webots::Motor *motor[20];  // motor
            webots::PositionSensor *positionSensor[20];  // position sensor
            webots::Accelerometer *accelerometer;  // accelerometer
            webots::Gyro *gyro;  // gyro

            double motorValue[20];  // motor control values
            double positionSensorValue[20];  // position sensor values
            // 加速度、および角速度センサは、lookupTableもoptionとして存在する。デフォルトは空（＝値は生データ）。
            const double *accelerometerValue;  // 3次元ベクトルで返ってくる。３要素であるため、indexは0~2に対応し、x,y,zの順。単位[m/s^2]。地球の重力加速度が乗っている。重力加速度は設定で変更可能。
            const double *gyroValue;  // 加速度センサと同様。単位[rad/s]
    };
}

#endif