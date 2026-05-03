import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Path to URDF inside human-gazebo package
    main_pkg = get_package_share_directory("argus_server")

    urdf_path = os.path.join(
        main_pkg,
        "urdf",
        "human.urdf"
    )

    # Load URDF into parameter
    with open(urdf_path, "r") as file:
        robot_description = file.read()

    joint_debug = LaunchConfiguration("joint_debug", default="false")

    declare_joint_debug = DeclareLaunchArgument(
        "joint_debug",
        default_value="false",
        description="Enable joint state publisher GUI"
    )


    node_ls = []
    node_ls.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description
            }]
        )
    )

    if joint_debug:
        node_ls.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher",
                output="screen",
                condition=IfCondition(joint_debug)
            )
        )
    node_ls.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', os.path.join(main_pkg, 'config', 'human_model.rviz')],
            output="screen"
        )
    )

    return LaunchDescription(node_ls)
