#include "rclcpp/rclcpp.hpp"
#include "kinematics/IK.hpp"

namespace kinematics {
  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSrv>());
    rclcpp::shutdown();
    return(0);
  }
}