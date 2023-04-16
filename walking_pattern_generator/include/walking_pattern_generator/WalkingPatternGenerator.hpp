#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/to_walking_stabilization_controller_message.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "Eigen/Dense"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      rclcpp::Publisher<msgs_package::msg::ToWalkingStabilizationControllerMessage>::SharedPtr toWSC_pub_;
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_IK_clnt_;

      // get FK, IK result. set publish data.
      std::array<double, 3> p_target_r_;
      std::array<double, 3> p_target_l_;
      std::array<double, 6> q_target_r_;
      std::array<double, 6> q_target_l_;

      // timer
      rclcpp::TimerBase::SharedPtr step_pub_;

      // PARAMETER
      bool publish_ok_check_;
      int step_counter_;
      int loop_number_;
      // 逆運動学からJointAngleを導出する
      std::array<std::array<double, 3>, 4> walking_pattern_P_R_;
      std::array<std::array<double, 3>, 4> walking_pattern_P_L_;
      std::array<std::array<double, 6>, 4> walking_pattern_jointVel_R_;
      std::array<std::array<double, 6>, 4> walking_pattern_jointVel_L_;

      void step_WPG_pub(void);

      void callback_res(const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future);

      Eigen::MatrixXd JacobiMatrix_calc();
      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_;
      std::array<Eigen::Vector3d, 6> UniVec_legR_;
      std::array<Eigen::Vector3d, 6> UniVec_legL_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}