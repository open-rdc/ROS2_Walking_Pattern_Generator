# Logger launch file

# TODO: rosbag2の仕様確認と、実行
  # 開発ソフトウェアに関わるTopicをrecord。他はいらん。recordするTopic名をconfig fileから読み取るか？User側で増やしたくなるかもしれんし、Configいじれば良いんだな！という思考の流れを持つだろうから。

# TODO: matlabとかで使えるような.dabとかに焼くLoggerNodeもほしいかも？
  # rosbag2のファイルは可読性がアレかもなので、飛んでいるTopicごとに別々に記録をしたい。

import os 
import datetime

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  launch_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

  #record_dir_path = "./src/ROS2_Walking_Pattern_Generator/Record/" + launch_datetime
  record_dir_path = "./src/Records/" + launch_datetime  # DEBUG

  try:
    os.mkdir(record_dir_path)
  except:
    print("Could not make record dir.")
    quit()

  record_dir_path = record_dir_path + "/"

  recorder_params = {
      "record_dir_path": record_dir_path, 
      "launch_datetime": launch_datetime
  }

  feedback_recorder_node = Node(
    package = "robot_recorder",
    executable = "feedback_recorder",
    parameters = [recorder_params],
  )

  footStep_recorder_node = Node(
    package = "robot_recorder",
    executable = "footStep_recorder",
    parameters = [recorder_params],
  )

  walkingPattern_recorder_node = Node(
    package = "robot_recorder",
    executable = "walkingPattern_recorder",
    parameters = [recorder_params],
  )

  walkingStabilization_recorder_node = Node(
    package = "robot_recorder",
    executable = "walkingStabilization_recorder",
    parameters = [recorder_params],
  )

  jointState_recorder_node = Node(
    package = "robot_recorder",
    executable = "jointState_recorder",
    parameters = [recorder_params],
  )


  return LaunchDescription([
    feedback_recorder_node,
    footStep_recorder_node,
    walkingPattern_recorder_node,
    walkingStabilization_recorder_node,
    jointState_recorder_node
  ])