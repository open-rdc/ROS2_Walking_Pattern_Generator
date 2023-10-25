#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "msgs_package/srv/to_kinematics_message.hpp"
#include <chrono>


using namespace std::chrono_literals;

namespace ik_client
{
  static const rmw_qos_profile_t custom_qos_profile =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // History: keep_last or keep_all
    1,  // History(keep_last) Depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // Reliability: best_effort or reliable
    RMW_QOS_POLICY_DURABILITY_VOLATILE,  // Durability: transient_local or volatile
    RMW_QOS_DEADLINE_DEFAULT,  // Deadline: default or number
    RMW_QOS_LIFESPAN_DEFAULT,  // Lifespan: default or number
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,  // Liveliness: automatic or manual_by_topic
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,  // Liveliness_LeaseDuration: default or number
    false  // avoid_ros_namespace_conventions
  };


  class  IKClient : public rclcpp::Node {
    public:
    IKClient(
      
    ) : Node("IKClient") {
      auto toKine_IK_clnt = this->create_client<msgs_package::srv::ToKinematicsMessage>("IK", custom_qos_profile);

      while(!toKine_IK_clnt->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "ERROR!!: IK service is dead.");
          return;
        }
        // RCLCPP_INFO(this->get_logger(), "Waiting for IK service...");
      }

      auto toKine_IK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage_Request>();

      toKine_IK_req->p_target_r = {-0.005, -0.037, -0.294056};
      toKine_IK_req->p_target_l = {0, 0, 0};
      toKine_IK_req->r_target_r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      toKine_IK_req->r_target_l = {1, 0, 0, 0, 1, 0, 0, 0, 1};

      auto toKine_IK_res = toKine_IK_clnt->async_send_request(
        toKine_IK_req, 
        [this](const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future) {
          std::cout << 
            "IK_result_R: "  <<
            future.get()->q_result_r[0] << " " <<
            future.get()->q_result_r[1] << " " <<
            future.get()->q_result_r[2] << " " <<
            future.get()->q_result_r[3] << " " <<
            future.get()->q_result_r[4] << " " <<
            future.get()->q_result_r[5] << std::endl;
          std::cout << 
            "IK_result_L: "  <<
            future.get()->q_result_l[0] << " " <<
            future.get()->q_result_l[1] << " " <<
            future.get()->q_result_l[2] << " " <<
            future.get()->q_result_l[3] << " " <<
            future.get()->q_result_l[4] << " " <<
            future.get()->q_result_l[5] << std::endl;
        }        
      );
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), toKine_IK_res);
    }
  };

}
  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ik_client::IKClient>());
    rclcpp::shutdown();
    
    return 0;
  }
