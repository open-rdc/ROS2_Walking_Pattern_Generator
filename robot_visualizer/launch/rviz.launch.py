import os
import yaml

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  launch_description = LaunchDescription()

  debug_mode_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_debug_mode.yaml")
  with open(debug_mode_yaml_path, "r") as f:
    debug_mode_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["setting_debug_mode"]
  
  description_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_robot_description.yaml")
  with open(description_yaml_path, "r") as f:
    description_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["robot_description"]
  
  # Get URDF via xacro
  robot_model = ParameterValue(
    Command([
      PathJoinSubstitution([FindExecutable(name="xacro")]),
      " ",
      PathJoinSubstitution(
        [FindPackageShare("robot_description"), "models/" + description_yaml["robot_name"] + "/" + description_yaml["xacro_file_path"]]
      ),
    ]), value_type=str
  )
  robot_model_param = {"robot_description": robot_model}
  
  rviz_config_file = PathJoinSubstitution([
    FindPackageShare("robot_visualizer"), "rviz", debug_mode_yaml["rviz_file"]
  ])
  
  robot_state_pub_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_model_param]
  )
  launch_description.add_action(robot_state_pub_node)
  
  if debug_mode_yaml["using_rviz"] == True:
    rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      arguments=["-d", rviz_config_file]
    )
    launch_description.add_action(rviz_node)
  

  return launch_description