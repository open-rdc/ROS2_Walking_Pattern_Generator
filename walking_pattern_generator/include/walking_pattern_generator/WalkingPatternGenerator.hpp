#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_walking_pattern_generator.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      void WPG_Server(
        const std::shared_ptr<msgs_package::srv::ToWalkingPatternGenerator::Request> request,
        std::shared_ptr<msgs_package::srv::ToWalkingPatternGenerator::Response> response
      );

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/

      rclcpp::Service<msgs_package::srv::ToWalkingPatternGenerator>::SharedPtr WPG_srv_;

      // 共有ライブラリの実体化
      kinematics::IK IK_;

      // 未使用
      // std::array<double, 3> p_target_r_;
      // std::array<double, 3> p_target_l_;
      // std::array<double, 6> q_target_r_;
      // std::array<double, 6> q_target_l_;

      int step_count_ = 0;

      // Parameterから受け取りたい（今はまだDEBUG_ParameterSetting()で定義）
      std::array<Eigen::Vector3d, 7> P_legR_; 
      std::array<Eigen::Vector3d, 7> P_legL_;

      std::array<std::array<double, 3>, 4> walking_pattern_P_R_;
      std::array<std::array<double, 3>, 4> walking_pattern_P_L_;
      std::array<std::array<double, 6>, 4> walking_pattern_jointVel_R_;
      std::array<std::array<double, 6>, 4> walking_pattern_jointVel_L_;
  };
}