"""Vision drift correction node. Optional: argus_server runs the same without it.

    ros2 launch argus_drift argus_drift.launch.py model:=rajagopal_lower pose_source:=mahoney period_s:=10
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = os.path.join(get_package_share_directory("argus_drift"), "config", "argus_drift.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rajagopal_upper",
                              description="model name under ~/ProjectArgus/models, or a folder"),
        DeclareLaunchArgument("pose_source", default_value="openvins", description="openvins or mahoney"),
        DeclareLaunchArgument("period_s", default_value="5.0", description="seconds between captures"),
        DeclareLaunchArgument("debug_dir", default_value="",
                              description="folder for match images and drift_log.jsonl (empty: none)"),
        Node(
            package="argus_drift",
            executable="drift_node",
            name="argus_drift",
            output="screen",
            parameters=[
                defaults,
                {
                    "model": LaunchConfiguration("model"),
                    "pose_source": LaunchConfiguration("pose_source"),
                    "period_s": ParameterValue(LaunchConfiguration("period_s"), value_type=float),
                    "debug_dir": LaunchConfiguration("debug_dir"),
                },
            ],
        ),
    ])
