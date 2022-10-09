#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace walking_pattern_generator {
    
    class WebotsRobotController : public webots_ros2_driver::PluginInterface {
        public:
            void step() override;
            void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    };
}