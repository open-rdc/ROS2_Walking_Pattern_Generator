#include "rclcpp/rclcpp.hpp"
#include "msgs_package/msg/walking_pattern.hpp"  // いずれ消える？
#include "msgs_package/msg/control_output.hpp"  // DEBUG:
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinematics/IK.hpp"
#include "kinematics/FK.hpp"
#include "kinematics/Jacobian.hpp"

#include "Eigen/Dense"

namespace walking_pattern_generator
{
  class WalkingPatternGenerator : public rclcpp::Node {
    public:
      WalkingPatternGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
      // publisher
      // TODO: managerが完成次第、ControlOutput -> WalkingPattern に変更するべき
      // TODO: 重要性からして、ここはServiceのほうがいい気がするんだ。
      // TODO: ここの型をJointStateにして、ros2_controlに対応させる。さすればRviz2との連携も可能。
      // rclcpp::Publisher<msgs_package::msg::ControlOutput>::SharedPtr pub_walking_pattern_;
      
      // CHECKME
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_walking_pattern_;

      // 共有ライブラリの実体化
      kinematics::FK FK_;
      kinematics::IK IK_;
      kinematics::Jacobian Jacobian_;

      // 動歩行パターン生成関数
      void WalkingPatternGenerate(void);

      // 歩行パターンの行列. 各関節の角度・回転速度を保存
      std::vector<std::array<double, 6>> WalkingPattern_Pos_legR_;
      std::vector<std::array<double, 6>> WalkingPattern_Pos_legL_;
      std::vector<std::array<double, 6>> WalkingPattern_Vel_legR_;
      std::vector<std::array<double, 6>> WalkingPattern_Vel_legL_;

      // ヤコビアン
      Eigen::Matrix<double, 6, 6> Jacobi_legR_;
      Eigen::Matrix<double, 6, 6> Jacobi_legL_;

      // 関節角度
      std::array<double, 6> Q_legR_;
      std::array<double, 6> Q_legL_;


// TODO: Parameterとして扱いたい変数達
      void DEBUG_ParameterSetting(void);

      float weight_;  // 重量[kg]
      float length_leg_;  // 脚の長さ[m]. これが腰の高さとなる。

      // 歩行パラメータ
      std::vector<std::array<double, 3>> LandingPosition_;

      // 目標足先姿勢行列
      Eigen::Matrix<double, 3, 3> R_target_leg;

      // 脚の単位行列
      std::array<Eigen::Vector3d, 6> UnitVec_legR_;
      std::array<Eigen::Vector3d, 6> UnitVec_legL_;

      // 脚のリンク長
        // proto準拠
      std::array<Eigen::Vector3d, 7> P_legR_;
      std::array<Eigen::Vector3d, 7> P_legL_;
        // 腰位置に合わせる
      std::array<Eigen::Vector3d, 7> P_legR_waist_standard_;
      std::array<Eigen::Vector3d, 7> P_legL_waist_standard_;
  };
}