#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"

#include "Eigen/Dense"

namespace kinematics
{
  class Default_ForwardKinematics : public control_plugin_base::ForwardKinematics {
    public:
      Default_ForwardKinematics(){}
      ~Default_ForwardKinematics(){}

      void forward_kinematics(
        std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
        Eigen::Vector3d& end_eff_pos_ptr
      ) override;
      void forward_kinematics(
        std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
        int joint_point,
        Eigen::Vector3d& end_eff_pos_ptr
      ) override;

      std::array<Eigen::Matrix3d, 6> getR_leg(
        std::array<double, 6> Q_leg
      );

    private:
      // TODO: これら汎用的なやつは、他のところに分けたい。これこそライブラリとかにしたい。
      Eigen::Matrix3d Rx(double rad = 0);  // ここ、Eigen系の関数として、また別に共有ライブラリを作りたい
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);
  };
}  

#endif 
