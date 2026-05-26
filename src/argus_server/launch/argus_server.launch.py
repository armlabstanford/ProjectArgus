from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os

#os.environ['LD_LIBRARY_PATH'] = (
#    '/home/argus/opensim-core/sdk/lib:/home/argus/opensim-core/sdk/Simbody/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
#)

def generate_launch_description():

    model_folder_path = '/home/argus/Argus/models/bimanual_upper_arm'
    topic_list = [
        '/thorax/odomimu',
        '/humerus_r/odomimu',
    ]

    joint_outputs = [
        "r_x",
        "r_y",
        "r_z",

        "elv_angle_r",
        "shoulder_elv_r",
        "shoulder_rot_r",
        "elbow_flexion_r",
        "pro_sup_r",
        "deviation_r",
        "flexion_r",

        "elv_angle_l",
        "shoulder_elv_l",
        "shoulder_rot_l",
        "elbow_flexion_l",
        "pro_sup_l",
        "deviation_l",
        "flexion_l",
    ]

    # This dictionary maps body names to their calibration euler angles [x, y, z].
    body_calibrations = {
        'thorax': [0.0, 0.0, 0.0],
        'humerus_r': [0.0, 0.0, 0.0],
    }

    sensor_weights = {
        'thorax': 40.0,
        'humerus_r': 1.0,
    }

    # The body_calibrations dictionary needs to be converted to a list of strings
    # to be passed as a ROS parameter. Each string is formatted as "key:value".
    body_calibrations_param = [f"{key}:{','.join(map(str, value))}" for key, value in body_calibrations.items()]
    sensor_weights_param = [f"{key}:{value}" for key, value in sensor_weights.items()]

    return LaunchDescription([

        # --- Node ---
        Node(
            package='argus_server',   # change if your package name differs
            executable='argus_server',
            name='argus_server',
            output='screen',
            parameters=[
                {
                    'model_folder_path': model_folder_path,
                    'topic_names': topic_list,
                    'joint_outputs': joint_outputs,
                    'body_calibrations': body_calibrations_param,
                    'sensor_weights': sensor_weights_param, 
                }
            ]
        )
    ])