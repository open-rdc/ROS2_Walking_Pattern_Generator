#include "rclcpp/rclcpp.hpp"

#include "walking_pattern_generator/Supervisor_pub.hpp"
#include <webots/Supervisor.hpp>

SupervisorPub::SupervisorPub(const rclcpp::NodeOptions &options)
: Node("Supervisor_pub", options) {
    // SUPERVISOR
    //supervisor = webots_ros2_driver::WebotsNode::mRobot();
    supervisor = new webots::Supervisor();
    SupervisorNode = supervisor->getFromDef("ROBOTIS_OP2");
    field_translation = SupervisorNode->getField("translation");
    field_rotation = SupervisorNode->getField("rotation");

    
    // SUPERVISOR: CHECK ROBOT DATA
    translation = field_translation->getSFVec3f();
    rotation = field_rotation->getSFRotation();

    std::cout << "field_trans: [" << translation[0] << ", " << translation[1] << ", " << translation[2] << "]" << std::endl;
    std::cout << "field_rotat: [" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << "]\n" << std::endl;
    

    delete supervisor;
}