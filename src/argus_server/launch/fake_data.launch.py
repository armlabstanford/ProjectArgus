from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    return LaunchDescription([

        # --- Node ---
        Node(
            package='argus_server',   # change if your package name differs
            executable='fake_data_gen',
            name='fake_data_gen',
            output='screen'
        )
    ])