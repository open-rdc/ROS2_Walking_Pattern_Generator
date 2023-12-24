#ifndef WALKING_STABILIZATION_CONTROLLER_HPP
#define WALKING_STABILIZATION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
// #include "robot_manager/control_plugin_bases/PluginBase_WalkingPatternGenerator.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_WalkingStabilizationController.hpp"

namespace walking_stabilization_controller
{
  class Default_WalkingStabilizationController : public control_plugin_base::WalkingStabilizationController
  {
    public:
      Default_WalkingStabilizationController();
      ~Default_WalkingStabilizationController(){}

      std::unique_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_controller(
        std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr
      ) override;
      
    private:
      rclcpp::Node::SharedPtr node_ptr_;
      std::shared_ptr<rclcpp::SyncParametersClient> client_param_;

  };
//   class Default_WalkingStabilizationController : public control_plugin_base::WalkingStabilizationController
//   {
//     public:
//       Default_WalkingStabilizationController(){}
//       ~Default_WalkingStabilizationController(){}

//       std::unique_ptr<control_plugin_base::WalkingStabilization> walking_stabilization_controller(
//         std::shared_ptr<control_plugin_base::WalkingPattern> walking_pattern_ptr,
//         const uint32_t control_step
//       ) override;
      
//     private:
//       uint32_t max_step = 0;
//   };
}

#endif