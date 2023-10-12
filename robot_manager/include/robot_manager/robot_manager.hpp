#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "Eigen/Dense"

namespace robot_manager
{
  class RobotManager {
    public:
      RobotManager();
    
    private:
      ~RobotManager(){}
  };
}