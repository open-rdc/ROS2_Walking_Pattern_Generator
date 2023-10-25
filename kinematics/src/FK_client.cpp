#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>
#include "msgs_package/srv/to_kinematics_message.hpp"

using namespace std::chrono_literals;

namespace fk_client
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


  class  FKClient : public rclcpp::Node {
    public:
    FKClient(
    
    ) : Node("FKClient") {
      auto toKine_FK_clnt = this->create_client<msgs_package::srv::ToKinematicsMessage>("FK", custom_qos_profile);

      while(!toKine_FK_clnt->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "ERROR!!: FK service is dead.");
          return;
        }
        // RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");
      }

      auto toKine_FK_req = std::make_shared<msgs_package::srv::ToKinematicsMessage_Request>();

      toKine_FK_req->q_target_r = {0, 0, -3.14/8, 3.14/4, -3.14/8, 0};
      toKine_FK_req->q_target_l = {0, 0, 0, 0, 0, 0};
      toKine_FK_req->fk_point = 6;

      auto toKine_FK_res = toKine_FK_clnt->async_send_request(
        toKine_FK_req, 
        [this](const rclcpp::Client<msgs_package::srv::ToKinematicsMessage>::SharedFuture future) {
          std::cout << 
            "FK_result_R: "  <<
            future.get()->p_result_r[0] << " " <<
            future.get()->p_result_r[1] << " " <<
            future.get()->p_result_r[2] << std::endl;
          std::cout << 
            "FK_result_L: "  <<
            future.get()->p_result_l[0] << " " <<
            future.get()->p_result_l[1] << " " <<
            future.get()->p_result_l[2] << std::endl;
        }        
      );
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), toKine_FK_res);
    }
  };

}
  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fk_client::FKClient>());
    rclcpp::shutdown();
    
    return 0;
  }
