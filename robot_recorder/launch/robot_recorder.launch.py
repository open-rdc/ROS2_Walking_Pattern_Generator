# Logger launch file

# TODO: rosbag2の仕様確認と、実行
  # 開発ソフトウェアに関わるTopicをrecord。他はいらん。recordするTopic名をconfig fileから読み取るか？User側で増やしたくなるかもしれんし、Configいじれば良いんだな！という思考の流れを持つだろうから。

import os 
import datetime
import yaml

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  launch_description = LaunchDescription()

  debug_mode_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_debug_mode.yaml")
  with open(debug_mode_yaml_path, "r") as f:
    debug_mode_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["setting_debug_mode"]

  launch_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

  #record_dir_path = "./src/ROS2_Walking_Pattern_Generator/Record/" + launch_datetime
  record_dir_path = "./src/Records/" + launch_datetime  # DEBUG

  try:
    os.mkdir(record_dir_path)
  except FileExistsError:
    pass
  except:
    print("Could not make record dir.")
    quit()

  record_dir_path = record_dir_path + "/"

  recorder_params = {
      "record_dir_path": record_dir_path, 
      "launch_datetime": launch_datetime
  }

  if debug_mode_yaml["using_feedback_recorder"] == True:
    feedback_recorder_node = Node(
      package = "robot_recorder",
      executable = "feedback_recorder",
      parameters = [recorder_params],
    )
    launch_description.add_action(feedback_recorder_node)

  if debug_mode_yaml["using_footStep_recorder"] == True:
    footStep_recorder_node = Node(
      package = "robot_recorder",
      executable = "footStep_recorder",
      parameters = [recorder_params],
    )
    launch_description.add_action(footStep_recorder_node)

  if debug_mode_yaml["using_walkingPattern_recorder"] == True:
    walkingPattern_recorder_node = Node(
      package = "robot_recorder",
      executable = "walkingPattern_recorder",
      parameters = [recorder_params],
    )
    launch_description.add_action(walkingPattern_recorder_node)

  if debug_mode_yaml["using_walkingStabilization_recorder"] == True:
    walkingStabilization_recorder_node = Node(
      package = "robot_recorder",
      executable = "walkingStabilization_recorder",
      parameters = [recorder_params],
    )
    launch_description.add_action(walkingStabilization_recorder_node)

  if debug_mode_yaml["using_jointState_recorder"] == True:
    jointState_recorder_node = Node(
      package = "robot_recorder",
      executable = "jointState_recorder",
      parameters = [recorder_params],
    )
    launch_description.add_action(jointState_recorder_node)

  if debug_mode_yaml["using_rosbag2"] == True:
    None


  return launch_description