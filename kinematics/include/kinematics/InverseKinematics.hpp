#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_InverseKinematics.hpp"

#include "Eigen/Dense"

namespace kinematics
{
  class Default_InverseKinematics : public control_plugin_base::InverseKinematics {
    public:
      Default_InverseKinematics(){}
      ~Default_InverseKinematics(){}

      void inverse_kinematics(
        const std::shared_ptr<control_plugin_base::LegStates_ToIK> leg_states_ptr,
        std::array<double, 6>& joint_ang_ptr
      ) override;

      // 便利関数として、他ライブラリにまとめたい
      Eigen::Vector3d Array2Vector(std::array<double, 3> array);  // std::array型をEigen::Vector3d型に変換（３次元）
      Eigen::Matrix3d Array2Matrix(std::array<double, 9> array);  // std::array型をEigen::Matrix3d型に変換（3*3行列）

    private:
      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix3d Rz(double rad = 0);
      Eigen::Matrix3d IdentifyMatrix(void);

      double sign(double arg = 0);  // return 1 or -1 (argが>=0なら1, <0なら-1を返す)

      const float pi_ = 3.141593;  // 四捨五入済み
    
    private:
  };

}  

#endif 
