#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/walking_pattern.hpp"
#include "kinematics/IK.hpp"

#include "Eigen/Dense"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      void WalkingPattern_Timer();

      rclcpp::Publisher<msgs_package::msg::WalkingPattern>::SharedPtr pub_walking_pattern_;
      rclcpp::TimerBase::SharedPtr timer_;

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

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}