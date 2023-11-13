import os

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  # load simulation launch file
  sim_launch = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("webots_robot_handler"), "launch"), 
      "/start_launch.py"
    ])
  )

  # TODO: Load Rviz2 launch file

  # TODO: Load Logger launch file

  # load robot_manager node
  robot_manager = Node(
    package = "robot_manager",
    executable = "robot_manager",
    output = "screen"
  )

  # Execution
  return launch.LaunchDescription([
    # another launch file
    sim_launch,

    # node
    robot_manager
  ])