#ifndef WALKING_PATTERN_GENERATOR_HPP
#define WALKING_PATTERN_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace walking_pattern_generator {
    // node_test1
    class Node_WalkingPatternGenerator : public rclcpp::Node {
        public:
            Node_WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr __pub;
    };

    class WalkingPatternGenerator : public webots_ros2_driver::PluginInterface {
        public:
            void init(
                webots_ros2_driver::WebotsNode *node, 
                std::unordered_map<std::string, std::string> &parameters
                ) override;
            void step() override;

            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr __pub;

        private:
            // node_test2
            // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
            webots_ros2_driver::WebotsNode *Node;
            webots::Robot *robot;
            webots::Motor *motor[20];
            webots::PositionSensor *positionSensor[20];
            webots::Accelerometer *accelerometer;
            webots::Gyro *gyro;
    };
}

#endif