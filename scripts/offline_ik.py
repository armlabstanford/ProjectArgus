# This script uses the opensim python implementation to perform offline orientation IK.
# supply a .csv of orientation data and requested joint angles, and this saves a csv
# of the resulting joint angles after IK.


import os
import opensim as osim
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D


# TODO: need to test everything with actual data!
# TODO: need to add visualization of IK results (doesn't work on ROG)



# quat conventions:
# scipy is x y z w
# opensim is w x y z
# vive is w x y z

def generate_sinusoid_data(num_samples:int, frequency:float, amplitude:float, central_point:float):
    """
    Generate a sinusoidal orientation trajectory for testing purposes. 
    """
    timestamps = [i * 0.01 for i in range(num_samples)]
    orientations = []
    for t in timestamps:
        angle = amplitude * np.sin(2 * np.pi * frequency * t)
        quat_1 = R.from_euler('xyz', [0, 0, central_point+angle]).as_quat()  # Convert to quaternion
        quat_2 = R.from_euler('xyz', [0, 0, central_point-angle]).as_quat()  # Convert to quaternion
        quat_3 = R.from_euler('xyz', [0, 0, 0]).as_quat()  # Convert to quaternion
        orientations.append((quat_1, quat_2, quat_3))
        print(quat_1)

    orientation_df = pd.DataFrame({
        'timestamp': timestamps,
        'left_thigh_q_x': [quat_row[0][0] for quat_row in orientations],
        'left_thigh_q_y': [quat_row[0][1] for quat_row in orientations],
        'left_thigh_q_z': [quat_row[0][2] for quat_row in orientations],
        'left_thigh_q_w': [quat_row[0][3] for quat_row in orientations],

        'right_thigh_q_x': [quat_row[1][0] for quat_row in orientations],
        'right_thigh_q_y': [quat_row[1][1] for quat_row in orientations],
        'right_thigh_q_z': [quat_row[1][2] for quat_row in orientations],
        'right_thigh_q_w': [quat_row[1][3] for quat_row in orientations],

        'torso_q_x': [quat_row[2][0] for quat_row in orientations],
        'torso_q_y': [quat_row[2][1] for quat_row in orientations],
        'torso_q_z': [quat_row[2][2] for quat_row in orientations],
        'torso_q_w': [quat_row[2][3] for quat_row in orientations],
    })

    return orientation_df

def prepare_model(model_folder_path:str):
    model_path = os.path.join(model_folder_path, 'model.osim')
    geom_path = os.path.join(model_folder_path, 'geometry')

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found at {model_path}")
    if os.path.exists(geom_path):
        osim.ModelVisualizer.addDirToGeometrySearchPaths(geom_path)

    model = osim.Model(model_path)
    
    state = model.initSystem()

    return model, state

def prepare_ik_solver(model:osim.Model, oRefs:osim.OrientationsReference, weights:dict,
                      constraint_var:float=10.0,
                      accuracy:float=0.0001):
    
    mRefs = osim.MarkersReference()
    coordinateReferences = osim.SimTKArrayCoordinateReference()

    ik_solver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordinateReferences, constraint_var)
    ik_solver.setAccuracy(accuracy)

    return ik_solver

def solve_ik(model:osim.Model, ik_solver:osim.InverseKinematicsSolver, state:osim.State, timestamps:list, desired_joints:list):

    coords = model.getCoordinateSet()
    ik_results = pd.DataFrame(columns = ["timestamp"] + desired_joints)
    ik_results["timestamp"] = timestamps

    for i in range(timestamps.shape[0]):

        state.setTime(timestamps[i])
        if i == 0:
            ik_solver.assemble(state)
        else:
            ik_solver.track(state)

        for j in range(coords.getSize()):
            coord = coords.get(j)
            joint_name = coord.getName()
            if joint_name not in desired_joints:
                continue
            ik_results.loc[i, joint_name] = coord.getValue(state)
        
    return ik_results

def df_to_orientation_refs(orientation_df:pd.DataFrame, model:osim.Model, initial_state:osim.State, 
                           zero_pose_timestamp:float, sensor_bone_offsets:dict, body_names:list, 
                           weights:dict,
                           data_label_to_osim:dict=None
                           ):
    """
    Expects a dataframe of format:
    timestamp, bodyname1_q_w, bodyname1_q_x, bodyname1_q_y, bodyname1_q_z, bodyname2_q_w...

    where bodyname1 is either the name of the osim body this coordinate refers to,
    or a custom label that is mapped to an osim body in the data_label_to_osim dict.

    This function also transforms sensor orientations to bone orientations, averaging the orientation in the zero_pose_timestamp_range,
    and finding the corresponding initial body transforms.
    """

    ext_list = ["_q_w", "_q_x", "_q_y", "_q_z"]

    if data_label_to_osim is not None:
        osim_body_names = [data_label_to_osim.get(label, label) for label in body_names]
    else:
        osim_body_names = body_names
    

    # extract the initial body poses
    model.realizePosition(initial_state)
    body_set = model.getBodySet()

    # get nearest index to zero_pose_timestamp
    zero_pose_idx = (orientation_df["timestamp"] - zero_pose_timestamp).abs().idxmin()
    
    ref_pose = {}

    for i, body_name in enumerate(body_names):
        T_GB = body_set.get(osim_body_names[i]).getTransformInGround(initial_state)
        R_GB = T_GB.R()

        R_GB = osim_rot_to_numpy(R_GB)

        odom_ref = np.array([orientation_df[body_name + ext].iloc[zero_pose_idx] for ext in ext_list])
        odom_ref = R.from_quat(odom_ref, scalar_first=True).as_matrix()
        sensor_offset = sensor_bone_offsets[body_name]

        ref_pose[body_name] = R_GB @ sensor_offset.T @ odom_ref.T
        print(body_name, "R_GB\n", R_GB, "\nsensor offset\n", sensor_offset, "\nodom ref\n", odom_ref, "\nref pose\n", ref_pose[body_name])

    # extract the sensor orientation at the zero pose timestamp, and compute the transforms we 
    # will apply to the rest of the orientation data.
    num_bodies = len(body_names)
    print(body_names)

    quatTable = osim.TimeSeriesTableQuaternion()
    quatTable.setColumnLabels(osim_body_names)

    for i, row in orientation_df.iterrows():
        rvq = osim.RowVectorQuaternion(num_bodies)
        for j, body_name in enumerate(body_names):
            cur_quat = np.array([row[body_name + ext] for ext in ext_list])
            #print(body_name, cur_quat)
            sensor_offset = sensor_bone_offsets[body_name]
            R_rel = sensor_offset.T @ R.from_quat(cur_quat, scalar_first=True).as_matrix().T @ ref_pose[body_name].T
            R_rel = R_rel.T
            quat_rel = R.from_matrix(R_rel).as_quat(scalar_first=True)  # Convert back to quaternion
            if i == 0:
                print(body_name, "relative rotmat at zero pose timestamp:\n", R_rel)

            updater = rvq.updElt(0, j)
            [updater.set(k, quat_rel[k]) for k in range(4)]
            
        quatTable.appendRow(row["timestamp"], rvq)
        
    # convert quat time series table to rotation matrix time series table
    rotTable = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
    
    orientation_weight_set = osim.OrientationWeightSet()
    [orientation_weight_set.cloneAndAppend(osim.OrientationWeight(body_name, weights[body_name])) for body_name in weights.keys()]

    oRefs = osim.OrientationsReference(rotTable, orientation_weight_set)

    return oRefs

def osim_rot_to_numpy(osim_R:osim.Rotation):
    R = osim_R.asMat33()   # SimTK Mat33
    R_np = np.array([
        [R.get(0,0), R.get(0,1), R.get(0,2)],
        [R.get(1,0), R.get(1,1), R.get(1,2)],
        [R.get(2,0), R.get(2,1), R.get(2,2)],
    ])

    return R_np

def plot_IK_results(ik_results:pd.DataFrame, desired_joints:list, size:tuple=None):
    import matplotlib.pyplot as plt
    import math

    num_joints = len(desired_joints)
    if num_joints == 0:
        print("No joints to plot.")
        return

    if size:
        rows, cols = size
        if rows * cols < num_joints:
            raise ValueError(f"Subplot grid size {size} is too small for {num_joints} joints.")
    else:
        rows = num_joints
        cols = 1

    fig, axes = plt.subplots(rows, cols, figsize=(cols * 6, rows * 4), squeeze=False)
    axes = axes.flatten()

    for i, joint in enumerate(desired_joints):
        ax = axes[i]
        ax.plot(ik_results["timestamp"], ik_results[joint] / np.pi * 180)
        ax.set_title(f"IK Result for {joint}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Joint Angle (deg)")
        ax.grid(True)

    # Hide unused subplots
    for i in range(num_joints, len(axes)):
        axes[i].set_visible(False)

    plt.tight_layout()
    plt.show()


def vive_dataloader(npz_path:str, des_ik_rate:float=100):
    """
    Loads our vive data into a pandas df for later processing.
    des_ik_rate denotes the frequency we should try to sample IK at.
    This is used to generate timestamps, where all values in the output df
    are the measurement at that sensor which occured most recently at the ik
    timestep.
    """
    ext_list = ["_q_w", "_q_x", "_q_y", "_q_z"]
    data = np.load(npz_path)
    keys = data.keys()
    for key in keys:
        print(key)

    body_names = [key.split("_quaternions")[0] for key in keys if "_quaternions" in key]

    print(body_names)


    first_timestamps = [data[body_name + "_unix_time_ns"][0] for body_name in body_names]
    start_time = max(first_timestamps)*1e-9  # convert to seconds

    last_timestamps = [data[body_name + "_unix_time_ns"][-1] for body_name in body_names]
    end_time = min(last_timestamps)*1e-9  # convert to seconds

    timestamp_col = np.arange(start_time, end_time, 1/des_ik_rate)  # generate timestamps at desired IK rate, in seconds
    df = pd.DataFrame(columns = ["timestamp"] + [body_name + ext for body_name in body_names for ext in ext_list])
    df["timestamp"] = timestamp_col

    for i, timestamp in enumerate(timestamp_col):
        for body_name in body_names:
            closest_idx = np.searchsorted(data[body_name + "_unix_time_ns"] * 1e-9, timestamp, side='right') - 1
            if closest_idx < 0:
                closest_idx = 0  # if timestamp is before the first measurement, use the first measurement
    
            for j, ext in enumerate(ext_list):
                df.loc[i, body_name + ext] = data[body_name + "_quaternions"][closest_idx][j]
    return df

def animate_vive_data(npz_path:str, sensor_bone_offsets: dict = None):
    """
    Display a playback of the vive data using matplotlib.
    """
    import matplotlib.pyplot as plt

    data = np.load(npz_path)
    keys = data.keys()
    body_names = list(set([key.split('_quaternions')[0] for key in keys if '_quaternions' in key]))

    # --- Data Synchronization ---
    first_timestamps = [data[body_name + "_unix_time_ns"][0] for body_name in body_names]
    start_time = max(first_timestamps) * 1e-9

    last_timestamps = [data[body_name + "_unix_time_ns"][-1] for body_name in body_names]
    end_time = min(last_timestamps) * 1e-9

    # Use a reasonable rate for animation, e.g., 30fps
    anim_rate = 30
    timestamps = np.arange(start_time, end_time, 1.0 / anim_rate)
    num_frames = len(timestamps)

    synced_data = {body: {'positions': np.zeros((num_frames, 3)), 'rotations': np.zeros((num_frames, 3, 3))} for body in body_names}

    # print out initial relative poses between sensors:
    for i in range(len(body_names)-1):
        body_name1 = body_names[i]
        body_name2 = body_names[i+1]
        pose1 = data[body_name1 + "_quaternions"][0]  
        pose2 = data[body_name2 + "_quaternions"][0]

        rel_mat = R.from_quat(pose1, scalar_first=True).as_matrix().T @ R.from_quat(pose2, scalar_first=True).as_matrix()
        rel_euler = R.from_matrix(rel_mat).as_euler('xyz', degrees=True)
        print(f"Initial relative orientation from {body_name1} to {body_name2} (degrees): {rel_euler}")

    for body_name in body_names:
        body_times = data[body_name + "_unix_time_ns"] * 1e-9
        body_pos = data[body_name + "_positions"]
        body_quat = data[body_name + "_quaternions"]

        initial_pose = R.from_quat(body_quat[0], scalar_first=True).as_matrix()

        # 

        for i, t in enumerate(timestamps):
            idx = np.searchsorted(body_times, t, side='right') - 1
            if idx < 0:
                idx = 0
            
            pos = body_pos[idx]
            #rot = R.from_quat(body_quat[idx], scalar_first=True)
            if sensor_bone_offsets is not None:
                rot = sensor_bone_offsets[body_name].T @ R.from_quat(body_quat[idx], scalar_first=True).as_matrix().T @ initial_pose @ sensor_bone_offsets[body_name]
                rot = rot.T
            else:
                rot = R.from_quat(body_quat[idx], scalar_first=True).as_matrix()
            
            #print(rot)

            synced_data[body_name]['positions'][i] = pos
            synced_data[body_name]['rotations'][i] = rot

    # --- Animation Setup ---
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Store plot objects (lines for axes)
    axis_lines = {body: [ax.plot([], [], [], '-')[0] for _ in range(3)] for body in body_names}
    colors = ['r', 'g', 'b'] # X, Y, Z

    def set_axes_equal(ax):
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        plot_radius = 0.5 * max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def init():
        all_pos = np.vstack([synced_data[body]['positions'] for body in body_names])
        max_range = (all_pos.max(axis=0) - all_pos.min(axis=0)).max() / 2.0
        mid = all_pos.mean(axis=0)
        
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Vive Tracker Animation')
        set_axes_equal(ax)
        return [line for lines in axis_lines.values() for line in lines]

    def update(frame):
        all_lines = []
        for body_name in body_names:
            origin = synced_data[body_name]['positions'][frame]
            rotation = synced_data[body_name]['rotations'][frame]
            
            for i in range(3): # For each axis (X, Y, Z)
                axis_vec = rotation[:, i] * 0.1 # 10cm long axes
                axis_end = origin + axis_vec
                
                lines = axis_lines[body_name]
                lines[i].set_data([origin[0], axis_end[0]], [origin[1], axis_end[1]])
                lines[i].set_3d_properties([origin[2], axis_end[2]])
                lines[i].set_color(colors[i])
                all_lines.append(lines[i])
        return all_lines

    ani = animation.FuncAnimation(fig, update, frames=num_frames,
                                  init_func=init, blit=True, interval=1000/anim_rate)

    plt.show()


if __name__ == "__main__":
    # sensor offset angles should be in the frame of the parent bone. Generally opensim has +z pointing to the right (from a first person perspective)
    #     +x point forward, and +y pointing up (feet to head)
    des_ik_rate = 100
    # animate_vive_data("data/walk_full_vid/walk_full_vid.npz")
    orientation_df = vive_dataloader("data/walk_full_vid/walk_full_vid.npz", des_ik_rate=des_ik_rate)



    #animate_vive_data("data/walk_full/walk_full.npz")
    #animate_vive_data("data/walk_full_vid/walk_full_vid.npz")

    model_folder = "models/gait2354"  # path to folder containing model.osim and geometry
    #orientation_csv = "path/to/orientation_data.csv"

    model, state = prepare_model(model_folder)
    #orientation_df = generate_sinusoid_data(num_samples=1000, 
    #                                        frequency=0.5, 
    #                                        amplitude=np.pi/4,
    #                                        central_point=0)  # Generate sinusoidal data with a central point of pi/4
    
    # in the data csv, I use left_arm_q_x... right_arm_q_x... torso_q_x...
    # but we can convert to opensim body names using this dict!

    # @ maisha you could also use "toes_r" if allowing the foot to bend.
    data_label_to_osim = {"right_foot": "calcn_r", 
                          "right_knee": "tibia_r", 
                          "waist": "femur_r"}
    
    desired_joints = ["knee_angle_r", "ankle_angle_r"]  # specify which joints we want to save from the IK results
    zero_pose_timestamp = orientation_df["timestamp"].iloc[0] + 0.2 # specify the timestamp in the orientation data that corresponds to the zero pose of the model
    

    # for setting offsets, imagine the series of rotations you would need to apply to the sensor, to get it to match the
    # osim axes (osim is y up, x forward, z to subject's righthand side)
    
    # offsets for walk_full:
    # sensor_bone_offsets = {
    #     "right_foot": R.from_euler('xyz', [0, 180, -15], degrees=True).as_matrix(),  
    #     "right_knee": R.from_euler('xyz', [-85, 0, 90], degrees=True).as_matrix(), 
    #     "waist": R.from_euler('xyz', [-90, 0, 90], degrees=True).as_matrix(), 
    # }

    # offsets for walk_full_vid:
    sensor_bone_offsets = {
        "right_foot": R.from_euler('xyz', [0, 180, -15], degrees=True).as_matrix(), 
        "right_knee": R.from_euler('xyz', [-90, 0, 0], degrees=True).as_matrix(), 
        "waist": R.from_euler('xyz', [-90, 0, 90], degrees=True).as_matrix(),
    }

    body_names = ["right_foot", "right_knee", "waist"]
    weights = {"right_foot": 0.4, "right_knee": 1.0, "waist": 5.0}

    # translate to osim names
    for body_name in data_label_to_osim.keys():
        weights[data_label_to_osim[body_name]] = weights[body_name]
        del weights[body_name]

    animate_vive_data("data/walk_full_vid/walk_full_vid.npz", sensor_bone_offsets=sensor_bone_offsets)

    oRefs = df_to_orientation_refs(orientation_df, model, state, 
                                   zero_pose_timestamp, sensor_bone_offsets, body_names,
                                    weights,
                                   data_label_to_osim=data_label_to_osim)
    
    
    ik_solver = prepare_ik_solver(model, oRefs, weights)

    # Perform IK and save results to CSV
    ik_results = solve_ik(model, ik_solver, state, orientation_df["timestamp"], desired_joints)

    # Plot the IK results
    plot_IK_results(ik_results, desired_joints, size=(2, 1))

    # wow it solves very fast, maybe track() is quicker with higher rate data? Oh and accuracy was
    # set to values from Patrick's script. Maybe we have too high of an accuracy requirement on 
    # the realtime case and could speed up tracking even more!
