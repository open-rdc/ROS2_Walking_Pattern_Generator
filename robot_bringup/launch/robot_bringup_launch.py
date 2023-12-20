import os
import yaml

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  launch_description = launch.LaunchDescription()

  mode_switch_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_mode_switch.yaml")
  with open(mode_switch_yaml_path, "r") as f:
    mode_switch_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["mode_switch"]

  # load simulation launch file
  if mode_switch_yaml["using_simulator"] == True:
    sim_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("webots_robot_handler"), "launch"), 
        "/start_launch.py"
      ])
    )
    launch_description.add_action(sim_launch)

  # Load visualizer launch file & recorder launch file
  if mode_switch_yaml["debug_mode"] == True:
    visual_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("robot_visualizer"), "launch"), 
        "/robot_visualizer.launch.py"
      ])
    )
    #launch_description.add_action(visual_launch)
    record_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("robot_recorder"), "launch"),
        "/robot_recorder.launch.py"
      ])
    )
    launch_description.add_action(record_launch)

  # Logger launch fileはRMの起動をしているlaunchで読み込み、起動をするべき。動いていないときにどう記録しろと。

  # load robot_manager node
  # TODO: RMの起動など、制御プログラムは他launchファイルに記述して、制御実行時にそのlaunchファイルを起動する手順を取るべき。
    # そうしないと、rosbagを使って再現するときに干渉しちゃうし、RMの起動を阻止しないと行けない。
  # robot_manager = Node(
  #   package = "robot_manager",
  #   executable = "robot_manager",
  #   output = "screen"
  # )
  # launch_description.add_action(robot_manager)

  # Execution
  return launch_description