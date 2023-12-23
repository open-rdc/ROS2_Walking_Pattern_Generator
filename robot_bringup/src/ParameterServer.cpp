// declare & get all_parameters
// Plugin用。PluginはLaunchで起動しないので、YAMLをパラメータとして受け取れない。ので、Serverを用意して、コンストラクタ内でパラメータを通信によって受け取ってもらう。

#include "rclcpp/rclcpp.hpp"

class ParameterServer : public rclcpp::Node {
  public:
    ParameterServer(
      const rclcpp::NodeOptions &options
    ) : Node("RobotParameterServer", options) {

      //robot_description
      robot_name_ = get_parameter("robot_description.robot_name").as_string();
      xacro_file_path_ = get_parameter("robot_description.xacro_file_path").as_string();
      urdf_file_path_ = get_parameter("robot_description.urdf_file_path").as_string();

      //control
      waist_pos_z_ = get_parameter("control_constant.waist_pos_z").as_double();
      control_cycle_ = get_parameter("control_times.control_cycle").as_double();
      walking_cycle_ = get_parameter("control_times.walking_cycle").as_double();
      both_leg_support_period_ = get_parameter("control_times.both_leg_support_period").as_double();
      single_leg_support_period_ = get_parameter("control_times.single_leg_support_period").as_double();

      // limb
      // left_leg_name_ = get_parameter(robot_name_+"_limb.limb_names.left_leg").as_string();
      // right_leg_name_ = get_parameter(robot_name_+"_limb.limb_names.right_leg").as_string();
      // left_leg_joint_names_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+left_leg_name_+".joint_names").as_string_array();
      // right_leg_joint_names_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+right_leg_name_+".joint_names").as_string_array();
      // left_leg_joint_numbers_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+left_leg_name_+".joint_numbers").as_integer_array();
      // right_leg_joint_numbers_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+right_leg_name_+".joint_numbers").as_integer_array();
      // left_leg_joint_unit_vecter_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+left_leg_name_+".joint_unit_vector").as_double_array();
      // right_leg_joint_unit_vecter_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+right_leg_name_+".joint_unit_vector").as_double_array();
      // left_leg_link_length_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+left_leg_name_+".link_length").as_double_array();
      // right_leg_link_length_ = get_parameter(robot_name_+"_limb.limb_without_fixed_joints."+right_leg_name_+".link_length").as_double_array();

      // name_lists
      all_joint_names_without_fixed_ = get_parameter(robot_name_+"_name_lists.all_names_without_fixed_joints.all_joint_names").as_string_array();
    }


  private:
    // param_fileに対応した変数

    // robot_description
    std::string robot_name_;
    std::string xacro_file_path_;
    std::string urdf_file_path_;

    // control
    double waist_pos_z_;
    double control_cycle_;
    double walking_cycle_;
    double both_leg_support_period_;
    double single_leg_support_period_;

    // limb
    // std::string left_leg_name_;
    // std::vector<std::string> left_leg_joint_names_;
    // std::vector<long> left_leg_joint_numbers_;
    // std::vector<double> left_leg_joint_unit_vecter_;
    // std::vector<double> left_leg_link_length_;
    // std::string right_leg_name_;
    // std::vector<std::string> right_leg_joint_names_;
    // std::vector<long> right_leg_joint_numbers_;
    // std::vector<double> right_leg_joint_unit_vecter_;
    // std::vector<double> right_leg_link_length_;

    // name_lists
    std::vector<std::string> all_joint_names_without_fixed_;


};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<ParameterServer>(opt));

  rclcpp::shutdown();

  return 0;
}