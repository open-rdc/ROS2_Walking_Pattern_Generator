import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
  package_dir = get_package_share_directory("webots_robot_handler")
  robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_robotis_op2_description.urdf')).read_text()

  # FK (kinematics)
  fk = Node(
    package = "kinematics",
    namespace = "walking_controller",  # 通信は、同namespace内でしか行えない
    executable = "fk_srv",  # CMakeLists.txtのtarget_nameに合わせる
    output = "screen",
  )

  # IK (kinematics)
  ik = Node(
    package = "kinematics",
    namespace = "walking_controller",
    executable = "ik_srv",
    output = "screen",
  )

  # walking_stabilization_controller
  walking_stabilization_controller = Node(
    package = "walking_stabilization_controller",
    namespace = "walking_controller",
    executable = "walking_stabilization_controller",
    output = "screen",
  )

  # webots world
  webots = WebotsLauncher(  
    world = os.path.join(package_dir, "worlds", "webots_simple_world.wbt")
  )

  # webots_robot_handler (C++_plugin of webots_ros2_driver)
  robotis_op2_driver = Node(
    package = "webots_ros2_driver",
    namespace = "walking_controller",
    executable = "driver",
    output = "screen",
    additional_env = {"WEBOTS_CONTROLLER_URL": "ipc://1234/ROBOTIS_OP2"},
    parameters = [
      {"robot_description": robot_description}
    ],
  )

  # walking_pattern_generator
  walking_pattern_generator = Node(
    package = "walking_pattern_generator",
    namespace = "walking_controller",
    executable = "walking_pattern_generator",
    output = "screen",
  )

  return launch.LaunchDescription([
    fk,
    ik,
    walking_stabilization_controller,
    webots,
    robotis_op2_driver,
    walking_pattern_generator,
    launch.actions.RegisterEventHandler(
      event_handler = launch.event_handlers.OnProcessExit(
        target_action = webots,
        on_exit = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
      )
    )        
  ])
