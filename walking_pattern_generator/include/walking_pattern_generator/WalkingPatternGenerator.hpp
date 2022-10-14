#ifndef WALKING_PATTERN_GENERATOR_HPP
#define WALKING_PATTERN_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace walking_pattern_generator {
    
    class WalkingPatternGenerator : public webots_ros2_driver::PluginInterface {
        public:
            void step() override;
            void init(
                webots_ros2_driver::WebotsNode *node, 
                std::unordered_map<std::string, std::string> &parameters
                ) override;
        private:
            webots_ros2_driver::WebotsNode *Node;
            webots::Robot *robot;
            webots::Motor *motor[20];
            webots::PositionSensor *positionSensor[20];
            webots::Accelerometer *accelerometer;
            webots::Gyro *gyro;



    };
}

#endif