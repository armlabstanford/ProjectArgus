from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import json
import os

#os.environ['LD_LIBRARY_PATH'] = (
#    '/home/argus/opensim-core/sdk/lib:/home/argus/opensim-core/sdk/Simbody/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
#)

MODELS_DIR = os.path.expanduser(os.environ.get('ARGUS_MODELS_DIR', '~/ProjectArgus/models'))


def model_folder(model):
    """A model name under MODELS_DIR, or a path to a model folder. ARGUS_MODEL_FOLDER overrides the default."""
    if model in ('bimanual_upper_arm', 'rajagopal_upper') and os.environ.get('ARGUS_MODEL_FOLDER'):
        return os.path.expanduser(os.environ['ARGUS_MODEL_FOLDER'])
    if os.sep in model or model.startswith('~'):
        return os.path.abspath(os.path.expanduser(model))
    return os.path.join(MODELS_DIR, model)


def generate_launch_description():

    def make_node(context):
        folder = model_folder(LaunchConfiguration('model').perform(context).strip())
        config_path = os.path.join(folder, 'argus_model.json')
        if not os.path.isfile(config_path):
            raise RuntimeError(f'no argus_model.json in {folder}')
        with open(config_path, encoding='utf-8') as handle:
            config = json.load(handle)
        # Paths in argus_model.json are relative to the repo root (models/..).
        repo = os.path.dirname(os.path.dirname(folder))

        # Incoming sensor topic name -> OpenSim body name, keyed by the first
        # element of the topic path (argus1 in /argus1/odomimu).
        modules = config['modules']
        source = LaunchConfiguration('pose_source').perform(context).strip().lower()
        if source in ('mahoney', 'mahony'):
            suffix = 'mahoney_odomimu'
        elif source in ('openvins', 'vio', 'odomimu'):
            suffix = 'odomimu_throttled'
        else:
            raise RuntimeError(
                f"pose_source must be openvins or mahoney, got {source!r}"
            )
        topic_list = [f'/{name}/{suffix}' for name in modules]
        print(f'argus_server model={config["name"]} ({folder}) pose_source={source}: '
              + ', '.join(f'{name}->{body}' for name, body in modules.items()))

        # Bone-to-sensor offsets as extrinsic xyz Euler angles in degrees, as
        # "body:x,y,z" strings. The newest calibration_*.json in calibration_dir
        # (written by pose_capture.py) replaces these defaults.
        body_calibrations_param = [f"{body}:{','.join(map(str, value))}"
                                   for body, value in config['body_calibrations'].items()]
        sensor_weights_param = [f"{body}:{value}" for body, value in config['sensor_weights'].items()]
        topic_translations_param = [f"{alias}:{body}" for alias, body in modules.items()]
        server = config.get('server', {})

        zero_pose_file = LaunchConfiguration('zero_pose_file').perform(context).strip() \
            or os.path.join(repo, config['poses_file'])
        zero_pose = LaunchConfiguration('zero_pose').perform(context).strip() or 'neutral'
        nodes = []
        if LaunchConfiguration('drift_correction').perform(context).strip().lower() in ('true', '1', 'yes'):
            from ament_index_python.packages import get_package_share_directory
            drift_defaults = os.path.join(get_package_share_directory('argus_drift'), 'config', 'argus_drift.yaml')
            nodes.append(Node(
                package='argus_drift',
                executable='drift_node',
                name='argus_drift',
                output='screen',
                parameters=[drift_defaults, {
                    'model': folder,
                    'pose_source': source,
                    'period_s': float(LaunchConfiguration('drift_period_s').perform(context)),
                }],
            ))
        return nodes + [Node(
            package='argus_server',
            executable='argus_server',
            name='argus_server',
            output='screen',
            parameters=[
                {
                    'model_folder_path': folder,
                    'topic_names': topic_list,
                    'joint_outputs': config['joint_outputs'],
                    'body_calibrations': body_calibrations_param,
                    'calibration_dir': os.path.join(repo, config['calibration_dir']),
                    'zero_pose_file': zero_pose_file,
                    'zero_pose': zero_pose,
                    'sensor_weights': sensor_weights_param,
                    'elv_angle_prior_weight': float(server.get('elv_angle_prior_weight', 0.0)),
                    'elv_angle_prior_deg': float(server.get('elv_angle_prior_deg', 0.0)),
                    'lock_coordinates': list(config.get('lock', [])),
                    'topic_translations': topic_translations_param,
                }
            ]
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='rajagopal_upper',
            description='Model folder name under ~/ProjectArgus/models (rajagopal_upper, rajagopal_lower) or a path; '
                        'its argus_model.json sets the modules, mounts, weights, joints, poses and calibrations',
        ),
        DeclareLaunchArgument(
            'pose_source',
            default_value='openvins',
            description='openvins uses odomimu_throttled; mahoney uses mahoney_odomimu',
        ),
        DeclareLaunchArgument(
            'zero_pose',
            default_value='neutral',
            description='Pose name in zero_pose_file that /reset_tracking zeroes against',
        ),
        DeclareLaunchArgument(
            'zero_pose_file',
            default_value='',
            description="Pose JSON (pose_capture.py format); empty uses the model's poses_file",
        ),
        DeclareLaunchArgument(
            'drift_correction',
            default_value='false',
            description='true also starts the argus_drift vision drift correction node (optional; '
                        'the server accepts its corrections whenever it runs)',
        ),
        DeclareLaunchArgument(
            'drift_period_s',
            default_value='5.0',
            description='seconds between drift correction captures',
        ),
        OpaqueFunction(function=make_node),
    ])
