#include "rclcpp/rclcpp.hpp"

#include "webots/Node.hpp"
#include "webots/Supervisor.hpp"

class SupervisorPub : public rclcpp::Node {
    public:
        SupervisorPub(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    private:
        // Supervisor(ただ、positionとかのグローバル座標系から見るやつは、Nodeからも見られそう。)
        webots::Node *SupervisorNode;  // getFromDef(const std::string &name(= robot name) );
        webots::Supervisor *supervisor;  // supervisor
        webots::Field *field_translation;  // ソースはロボットのやつで使う。
        webots::Field *field_rotation;
        // Reference: https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_node_get_contact_points
        webots::ContactPoint *contactPoint;  // ContactPointを取得するために必要。getContactPoints. 事前にtrackingをenableにする必要有り。

        // ソースはロボット。以下どちらも、ロボットの座標軸の原点基準で出される。正確な重心ではない。
        const double *translation;  // supervisor: getSFVec3f(): robot's translation
        const double *rotation;  // supervisor: getRotation(): robot's rotation

        // Reference: https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_node_get_position
        // ソースはグローバル座標系。
        const double *position;  // グローバル座標系から見たロボットのローカル座標系の位置。
        const double *orientation;  // 回転行列
        const double *pose; // 位置ベクトルと回転行列を含んだ、同次変換行列。事前にtrackingをenableにする必要有り。
        const double *centerOfMass;  // グローバル座標系から見た、ロボットの重心位置

        const double *velocity;  // グローバル座標系でのロボットの速度。線形速度と角速度、計６要素の配列。

};