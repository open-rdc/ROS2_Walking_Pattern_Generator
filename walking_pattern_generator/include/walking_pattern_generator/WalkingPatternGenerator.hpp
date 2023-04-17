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
      rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedPtr toKine_FK_clnt_;
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
      std::array<double, 5> walking_pattern_x_;
      std::array<double, 5> walking_pattern_y_;
      Eigen::Vector3d walking_pattern_jointVel_x_;
      Eigen::Vector3d walking_pattern_jointVel_y_;

      void step_WPG_pub(void);

      // Lamdaに移行
      // void callback_FK_res(const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future);
      // void callback_IK_res(const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future);

      std::array<double, 6> Vector2Array(Eigen::Vector<double, 6> vector);

      void JacobiMatrix_leg(std::array<double, 6> Q_legR, std::array<double, 6> Q_legL);
      Eigen::Matrix<double, 6, 6> Jacobi_legR_;
      Eigen::Matrix<double, 6, 6> Jacobi_legL_;
      std::array<Eigen::Vector3d, 6> P_FK_legR_;
      std::array<Eigen::Vector3d, 6> P_FK_legL_;
      std::array<Eigen::Vector3d, 6> UnitVec_legR_;
      std::array<Eigen::Vector3d, 6> UnitVec_legL_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}