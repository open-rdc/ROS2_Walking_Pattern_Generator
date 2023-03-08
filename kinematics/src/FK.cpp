#include "rclcpp/rclcpp.hpp"
#include "msgs_package/srv/to_kinematics_message.hpp"
#include "Kinematics/FK.hpp"

#include "iostream"
#include "cmath"
#include "Eigen/Dense"

#include <string>

namespace Kinematics
{
  using namespace FKSrv;
  using namespace Eigen;

  // 3D Rotation Matrix
  Matrix3d IdentifyMatrix() {
    Matrix3d I;
    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(I);
  }
  Matrix3d Rx(double rad = 0) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d Ry(double rad = 0) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix3d Rz(double rad = 0) {
    Matrix3d R_z;
    R_z << cos(rad), -sin(rad), 0,
           sin(rad),  cos(rad), 0,
                  0,         0, 1;
    return(R_z);
  }

  void DEBUG_ParameterSetting() {
    P_legL = {(-0.005, 0.037, -0.1222),
         (0, 0, 0),
         (0, 0, 0),
         (0, 0, -0.093),
         (0, 0, -0.093),
         (0, 0, 0),
         (0, 0, 0)
    };
    P_legR = {(-0.005, -0.037, -0.1222),
         (0, 0, 0),
         (0, 0, 0),
         (0, 0, -0.093),
         (0, 0, -0.093),
         (0, 0, 0),
         (0, 0, 0)
    };
  }

  // Service Server
  void FK_SrvServer(
    const std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Request> request,
    std::shared_ptr<msgs_package::srv::ToKinematicsMessage::Response> response
  ) {
    // DEBUG=====
    rad_leg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // requestから受け取りたい
    std::string LorR_CF("LR");  // requestから受け取りたい。LR = Leg_Right, LL = Leg_Left, AR = Arm_Right, AL = Arm_Left.
    Vector3d FK_result;
    // DEBUG=====

    R = {Rz(rad_leg[0]), Rx(rad_leg[1]), Ry(rad_leg[2]), Ry(rad_leg[3]), Ry(rad_leg[4]), Rx(rad_leg[5])};

    if(LorR_CF == "LR") {
      FK_result = R[0] * R[1] * R[2] * R{3} * R[4] * R[5] * P_legR[6]
                  + R[0] * R[1] * R[2] * R[3] * R[4] * P_legR[5]
                  + R[0] * R[1] * R[2] * R[3] * P_legR[4]
                  + R[0] * R[1] * R[2] * P_legR[3]
                  + R[0] * R[1] * P_legR[2]
                  + R[0] * P_legR[1]
                  + P_legR[0];
    } 
    else if(LorR_CF == "LL") {
        FK_result = R[0] * R[1] * R[2] * R{3} * R[4] * R[5] * P_legL[6]
                  + R[0] * R[1] * R[2] * R[3] * R[4] * P_legL[5]
                  + R[0] * R[1] * R[2] * R[3] * P_legL[4]
                  + R[0] * R[1] * R[2] * P_legL[3]
                  + R[0] * R[1] * P_legL[2]
                  + R[0] * P_legL[1]
                  + P_legL[0];
    }

  }

  // Node Setting
  FKSrv(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  ) : Node("FK", options) {
    using namespace std::placeholders;

    DEBUG_ParameterSetting();
    
    toKine_srv_ptr = this->create_service<msgs_package::srv::ToKinematicsMessage>(
      "FK_SrvServer", 
      std::bind(&FKSrv::FK_SrvServer, this, _1, _2)
    );
  }
}