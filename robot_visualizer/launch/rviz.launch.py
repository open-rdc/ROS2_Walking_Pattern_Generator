from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    model_packages_name = "robot_description"
    visual_package_name = "robot_visualizer"
    xacro_file_name = "robotis_op2.urdf.xacro"
    rviz_file_name = "display.rviz"

    # Get URDF via xacro
    robot_model = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(model_packages_name), "models/robotis_op2/urdf", xacro_file_name]
            ),
        ]
    ), value_type=str)
    robot_model_param = {"robot_description": robot_model}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(visual_package_name), "rviz", rviz_file_name]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_model_param],
    )
    # joint_state_pub_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     output="screen",
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # my_joint_states_node = Node(
    #     package="my_joint_states",
    #     executable="my_joint_states",
    #     name="my_joint_states",
    #     output="screen",
    # )

    nodes = [
        rviz_node,
        robot_state_pub_node,
        # joint_state_pub_gui_node,
        #my_joint_states_node,
    ]

    return LaunchDescription(nodes)