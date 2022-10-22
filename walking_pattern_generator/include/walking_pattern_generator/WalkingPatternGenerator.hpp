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

            // Node
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



            // Supervisor(ただ、positionとかのグローバル座標系から見るやつは、Nodeからも見られそう。)
            webots::Node *SupervisorNode;  // getFromDef(const std::string &name(= robot name) );
            webots::Supervisor *supervisor;  // supervisor
            webots::Field *field_translation;  // ソースはロボットのやつで使う。
            webots::Field *field_rotation;
            // Reference: https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_node_get_contact_points
            webots::ContactPoint *contactPoint;  // ContactPointを取得するために必要。getContactPoints. 事前にtrackingをenableにする必要有り。

            // ソースはロボット。以下どちらも、ロボットの座標軸の原点基準で出される。正確な重心ではない。
            const double *translation;  // supervisor: getSFVec3f(): robot's translation
            const double *rotation;  // supervisor: getRotation(): robot's rotation

            // Reference: https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_node_get_position
            // ソースはグローバル座標系。
            const double *position;  // グローバル座標系から見たロボットのローカル座標系の位置。
            const double *orientation;  // 回転行列
            const double *pose; // 位置ベクトルと回転行列を含んだ、同次変換行列。事前にtrackingをenableにする必要有り。
            const double *centerOfMass;  // グローバル座標系から見た、ロボットの重心位置

            const double *velocity;  // グローバル座標系でのロボットの速度。線形速度と角速度、計６要素の配列。
    };
}

#endif