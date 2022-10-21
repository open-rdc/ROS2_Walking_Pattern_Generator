
import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

def generate_launch_description():
    package_dir = get_package_share_directory("walking_pattern_generator")
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_robotis_op2_description.urdf')).read_text()

    webots = WebotsLauncher(
        world = os.path.join(package_dir, "worlds", "webots_simple_world.wbt")
    )

    # robotis_op2_driver = Node(
    #     package = "webots_ros2_driver",
    #     executable = "driver",
    #     output = "screen",
    #     additional_env = {"WEBOTS_CONTROLLER_URL": "ipc://1234/ROBOTIS_OP2"},
    #     parameters = [
    #         {"robot_description": robot_description}
    #     ],
    # )
    supervisor_pub = Node(
        package = "walking_pattern_generator",
        executable = "supervisor_pub",
        output = "screen",
        additional_env = {"WEBOTS_CONTROLLER_URL": "ipc://1234/robot"},
        parameters = [
            {"robot_description": robot_description}
        ],
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    return launch.LaunchDescription([
        webots,
        # robotis_op2_driver,
        supervisor_pub,
        ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = webots,
                on_exit = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
            )
        )
        
        
    ])
