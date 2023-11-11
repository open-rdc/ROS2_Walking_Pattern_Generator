import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
  package_dir = get_package_share_directory("webots_robot_handler")
  robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_robotis_op2_description.urdf')).read_text()
  # webots world
  webots = WebotsLauncher(  
    world = os.path.join(package_dir, "worlds", "webots_simple_world.wbt")
  )

  # webots_robot_handler (C++_plugin of webots_ros2_driver)
  # TODO: 書き方が古くてWarningが出てる。WebotsControllerを使う方法に書き換えるべき。
    # ref: https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_mavic
  robotis_op2_driver = Node(
    package = "webots_ros2_driver",
    executable = "driver",
    output = "screen",
    additional_env = {"WEBOTS_CONTROLLER_URL": "ipc://1234/ROBOTIS_OP2"},
    parameters = [
      {"robot_description": robot_description},
      {'use_sim_time': True}
    ]
  )

  return launch.LaunchDescription([
    webots,
    robotis_op2_driver,
    launch.actions.RegisterEventHandler(
      event_handler = launch.event_handlers.OnProcessExit(
        target_action = webots,
        on_exit = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
      )
    )        
  ])


""" WARNING LOG
[driver-3] [WARN] [1693453176.235405393] [ROBOTIS_OP2]: Passing robot description as a string is deprecated. Provide the URDF or Xacro file path instead.
[ドライバー-3] [警告] [1693453176.235405393] [ROBOTIS_OP2]: ロボットの説明を文字列として渡すことは非推奨です。 代わりに URDF または Xacro ファイル パスを指定します。

[driver-3] [WARN] [1693453176.244747602] [ROBOTIS_OP2]: The direct declaration of the driver node in the launch file is deprecated. Please use the new WebotsController node instead.
[ドライバー-3] [警告] [1693453176.244747602] [ROBOTIS_OP2]: 起動ファイル内のドライバー ノードの直接宣言は非推奨です。 代わりに新しい WebotsController ノードを使用してください。
"""