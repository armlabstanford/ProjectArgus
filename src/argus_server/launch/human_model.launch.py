import json
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _hide_link(name):
    """Legs, feet, the muscles that cross them, and the hands."""
    leg_markers = (
        "Leg", "Foot", "Toe", "FTSensor",
        "RecFem", "BicFem", "TibAnt", "GasLat", "GasMed",
    )
    if any(marker in name for marker in leg_markers):
        return True
    if "Hand" in name:
        return True
    return name.endswith("_RH") or name.endswith("_LH")


def _upper_body_urdf(xml_text):
    root = ET.fromstring(xml_text)
    drop = {
        link.get("name")
        for link in root.findall("link")
        if link.get("name") and _hide_link(link.get("name"))
    }
    for link in list(root.findall("link")):
        if link.get("name") in drop:
            root.remove(link)
    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link") if parent is not None else ""
        child_name = child.get("link") if child is not None else ""
        if parent_name in drop or child_name in drop:
            root.remove(joint)
    return ET.tostring(root, encoding="unicode")


MODELS_DIR = os.path.expanduser(os.environ.get("ARGUS_MODELS_DIR", "~/ProjectArgus/models"))


def _urdf_name(model):
    """The URDF named in the model's argus_model.json (human.urdf if there is none)."""
    folder = os.path.expanduser(model) if (os.sep in model or model.startswith("~")) else os.path.join(MODELS_DIR, model)
    config = os.path.join(folder, "argus_model.json")
    if not os.path.isfile(config):
        return "human.urdf"
    with open(config, encoding="utf-8") as handle:
        return json.load(handle).get("urdf", "human.urdf")


def _launch_setup(context, *args, **kwargs):
    main_pkg = get_package_share_directory("argus_server")
    urdf_path = os.path.join(main_pkg, "urdf", _urdf_name(LaunchConfiguration("model").perform(context).strip()))
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    if LaunchConfiguration("upper_body_only").perform(context) == "true":
        robot_description = _upper_body_urdf(robot_description)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("joint_debug")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(main_pkg, "config", "human_model.rviz")],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="rajagopal_upper",
            description="Model folder name under ~/ProjectArgus/models; its argus_model.json names the URDF",
        ),
        DeclareLaunchArgument(
            "joint_debug",
            default_value="false",
            description="Enable joint state publisher GUI",
        ),
        DeclareLaunchArgument(
            "upper_body_only",
            default_value="false",
            description="Hide the legs and hands in RViz",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
