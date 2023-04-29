import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
  package_dir = get_package_share_directory("webots_robot_handler")
  robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_robotis_op2_description.urdf')).read_text()
  # robotis_op2_ros2_control_params = os.path.join(package_dir, "resource", "robotis_op2_ros2_control.yaml")

  # FK (kinematics)
  fk = Node(
    package = "kinematics",
    executable = "fk_srv",  # CMakeLists.txtのtarget_nameに合わせる
    output = "screen",
    parameters = [{'use_sim_time': True}]
  )

  # IK (kinematics)
  ik = Node(
    package = "kinematics",
    executable = "ik_srv",
    output = "screen",
    parameters = [{'use_sim_time': True}]
  )

  # # walking_stabilization_controller
  # walking_stabilization_controller = Node(
  #   package = "walking_stabilization_controller",
  #   executable = "walking_stabilization_controller",
  #   output = "screen",
  #   parameters = [{'use_sim_time': True}]
  # )

  # webots world
  webots = WebotsLauncher(  
    world = os.path.join(package_dir, "worlds", "webots_simple_world.wbt")
  )

  # webots_robot_handler (C++_plugin of webots_ros2_driver)
  robotis_op2_driver = Node(
    package = "webots_ros2_driver",
    executable = "driver",
    output = "screen",
    additional_env = {"WEBOTS_CONTROLLER_URL": "ipc://1234/ROBOTIS_OP2"},
    parameters = [
      {"robot_description": robot_description},
      {'use_sim_time': True}
      # robotis_op2_ros2_control_params
    ]
  )

  # walking_pattern_generator
  walking_pattern_generator = Node(
    package = "walking_pattern_generator",
    executable = "walking_pattern_generator",
    output = "screen",
    parameters = [{'use_sim_time': True}]
  )

  return launch.LaunchDescription([
    fk,
    ik,
    # walking_stabilization_controller,
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
