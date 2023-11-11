import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  sim_launch_share_dir = get_package_share_directory("webots_robot_handler")

  sim_launch = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource([sim_launch_share_dir + "/launch/start_launch.py"])
  )

  return launch.LaunchDescription([
    sim_launch
  ])