# Visualizer launch file
import os

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
# Rviz2のlaunch
  rviz_launch = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("robot_visualizer"), "launch"),
      "/rviz.launch.py"
    ])
  )

# Loggerとの違いは、リアルタイムに表示をするだけ　｜　記録を取る

# TODO: リアルタイムのモニタリングNodeとかの実行（作れれば。rqt、imgui,etc?）
# ros関連を詳しく出してくれるtopを実行するのも良さげ？
  # これ：https://github.com/iwatake2222/rotop
    # TODO: 動作確認と、pythonソースから実行できるか確認（コマンドラインでも、pythonでも）
  
  return launch.LaunchDescription([
    rviz_launch
  ])