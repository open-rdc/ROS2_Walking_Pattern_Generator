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


      // PARAMETER
      bool publish_ok_check_;
      int step_counter_;
      int loop_number_;
      
      // parameter 
      Eigen::Vector2d init_com_;  // 初期重心位置(x, y)
      double init_com_z_;  // 必要？いらないはず。 一応、記録用。
      // parameter
      Eigen::Matrix<double, 2, 5> walking_pattern_s_;  // 歩行パラメータ(x1~5, y1~5)
      // parameter
      int t_sup_;
      
      // calc
      Eigen::Vector2d init_global_p_;  // 初期着地位置(x, y)
      Eigen::Vector2d global_p_;  // 着地位置(x, y)
      Eigen::Vector2d global_p_pre_;  // １step前の着地位置(x, y)
      
      Eigen::Vector2d segment_;  // 歩行素片位置(x, y)
      Eigen::Vector2d segment_vel_;  // 歩行素片速度(x, y)
      
      Eigen::Vector2d x_;  // n歩目の歩行状態のxの最終状態(x, dx)
      Eigen::Vector2d y_;  // 上に同じ(y, dy)
      
      Eigen::Vector2d xi_;  // n歩目が始まる瞬間のxの重心位置と速度(x, dx)
      Eigen::Vector2d yi_;  // 上に同じ(y, dy)
      
      Eigen::Vector2d target_x_;  // xの最終状態の目標値(x, dx)
      Eigen::Vector2d target_y_;  // 上に同じ(y, dy)




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

      std::array<std::array<double, 3>, 4> walking_pattern_P_R_;
      std::array<std::array<double, 3>, 4> walking_pattern_P_L_;
      std::array<std::array<double, 6>, 4> walking_pattern_jointVel_R_;
      std::array<std::array<double, 6>, 4> walking_pattern_jointVel_L_;

// DEBUG===/*
      void DEBUG_ParameterSetting(void);
// DEBUG===*/
  };
}