#ifndef KINEMATICS__IK_HPP_
#define KINEMATICS__IK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "kinematics/visibility_control.h"

namespace kinematics
{
  class IK : public rclcpp::Node {
    public:
      IK(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  
  };

}  

#endif 
