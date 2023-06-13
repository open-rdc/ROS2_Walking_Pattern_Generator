#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "walking_stabilization_controller/WalkingStabilizationController.hpp"
#include "msgs_package/srv/stabilization_control.hpp"
#include "kinematics/FK.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

using namespace Eigen;

namespace robot_manager
{
  static const rmw_qos_profile_t custom_qos_profile =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
    1,  // History(keep_last) Depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
    RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
    RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
    RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
    false  // avoid_ros_namespace_conventions
  };

  void WalkingStabilizationController::DEBUG_ParameterSetting() {
    P_legL_ = {
        Vector3d(-0.005, 0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
    P_legR_ = {
        Vector3d(-0.005, -0.037, -0.1222),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, -0.093),
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 0)
    };
  }

  WalkingStabilizationController::WalkingStabilizationController(
    const rclcpp::NodeOptions &options
  ) : Node("WalkingStabilizationController", options) {

    using namespace std::placeholders;

    srv_stabilization_control_ = this->create_service<msgs_package::srv::StabilizationControl>(
      "StabilizationControl",
      std::bind(&WalkingStabilizationController::WSC_Server, this, _1, _2, _3),
      custom_qos_profile
    );
  }

  void WalkingStabilizationController::WSC_Server(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<msgs_package::srv::StabilizationControl::Request> request,
    std::shared_ptr<msgs_package::srv::StabilizationControl::Response> response
  ) {
    RCLCPP_INFO(this->get_logger(), "WalkingStabilizationController::WSC_Server");
    (void)request_header;
    // (void)request;
    // (void)response;
    // ふざけたコード
    response->nini = 99;
    response->q_next_leg_r = {99, 99, 99, 99, 99, 99};
    response->q_next_leg_l = request->q_target_leg_l;
    // response->dq_next_leg_r = request->dq_target_leg_r;
    // response->dq_next_leg_l = request->dq_target_leg_l;
    RCLCPP_INFO(this->get_logger(), "Response");
  }
}
