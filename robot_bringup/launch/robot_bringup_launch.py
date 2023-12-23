import os
import yaml
from time import sleep

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  launch_description = launch.LaunchDescription()

  mode_switch_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_mode_switch.yaml")
  with open(mode_switch_yaml_path, "r") as f:
    mode_switch_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["mode_switch"]
  robot_description_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_robot_description.yaml")
  with open(robot_description_yaml_path, "r") as f:
    robot_description_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["robot_description"]

  # load param_file
  param_mode_switch_yaml = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_mode_switch.yaml")
  param_control_yaml = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_control.yaml")
  param_robot_description_yaml = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_robot_description.yaml")
  limb_yaml = os.path.join(get_package_share_directory("robot_description"), "config", robot_description_yaml["robot_name"], "param_"+robot_description_yaml["robot_name"]+"_limb.yaml")
  name_list_yaml = os.path.join(get_package_share_directory("robot_description"), "config", robot_description_yaml["robot_name"], "param_"+robot_description_yaml["robot_name"]+"_name_lists.yaml")

    # load param server
  parameter_server_node = Node(
    package = "robot_bringup",
    executable = "robot_parameter_server",
    output = "screen",
    parameters = [
      param_mode_switch_yaml,
      param_control_yaml,
      param_robot_description_yaml,
      limb_yaml,
      name_list_yaml
    ]
  )
  launch_description.add_action(parameter_server_node)
  # sleep(2)

  # load robot_manager node
  # TODO: RMの起動など、制御プログラムは他launchファイルに記述して、制御実行時にそのlaunchファイルを起動する手順を取るべき。
    # そうしないと、rosbagを使って再現するときに干渉しちゃうし、RMの起動を阻止しないと行けない。
  robot_manager = Node(
    package = "robot_manager",
    executable = "robot_manager",
    output = "screen"
  )
  launch_description.add_action(robot_manager)
  # sleep(1)

  # Load visualizer launch file & recorder launch file
  if mode_switch_yaml["debug_mode"] == True:

    record_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("robot_recorder"), "launch"),
        "/robot_recorder.launch.py"
      ])
    )
    launch_description.add_action(record_launch)

    visual_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("robot_visualizer"), "launch"), 
        "/robot_visualizer.launch.py"
      ])
    )
    launch_description.add_action(visual_launch)

  # load simulation launch file
  if mode_switch_yaml["using_simulator"] == True:
    sim_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("webots_robot_handler"), "launch"), 
        "/start_launch.py"
      ])
    )
    launch_description.add_action(sim_launch)


  # Execution
  return launch_description