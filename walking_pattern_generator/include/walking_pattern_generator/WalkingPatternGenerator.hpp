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
      
      // parameter 
      Eigen::Vector2d walking_pattern_init_com_;  // 初期重心位置(x, y)
      double init_com_z_;  // 必要？いらないはず。 一応、記録用。
      // parameter
      Eigen::Matrix<double, 2, 5> walking_pattern_s_;  // 歩行パラメータ(x1~5, y1~5)
      
      // calc
      Eigen::Vector2d walking_pattern_init_p_;  // 初期着地位置(x, y)
      Eigen::Vector2d walking_pattern_p_;  // 着地位置(x, y)
      Eigen::Vector2d walking_pattern_p_old_;  // １step前の着地位置(x, y)
      
      Eigen::Vector2d walking_segment_;  // 歩行素片位置(x, y)
      Eigen::Vector2d walking_segment_vel_;  // 歩行素片速度(x, y)
      
      Eigen::Vector2d walking_pattern_x_;  // n歩目の歩行状態のxの最終状態(x, dx)
      Eigen::Vector2d walking_pattern_y_;  // 上に同じ(y, dy)
      
      Eigen::Vector2d walking_pattern_xi_;  // n歩目が始まる瞬間のxの重心位置と速度(x, dx)
      Eigen::Vector2d walking_pattern_yi_;  // 上に同じ(y, dy)
      
      Eigen::Vector2d walking_pattern_target_x_;  // xの最終状態の目標値(x, dx)
      Eigen::Vector2d walking_pattern_target_y_;  // 上に同じ(y, dy)




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