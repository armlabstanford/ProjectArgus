
# This script uses the opensim python implementation to perform offline orientation IK.
# supply a .csv of orientation data and requested joint angles, and this saves a csv
# of the resulting joint angles after IK.


import os
import opensim as osim
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R


# TODO: need to test everything with actual data!
# TODO: need to add visualization of IK results (doesn't work on ROG)

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

    orientation_df = pd.DataFrame({
        'timestamp': timestamps,
        'left_thigh_q_x': [quat_row[0][1] for quat_row in orientations],
        'left_thigh_q_y': [quat_row[0][2] for quat_row in orientations],
        'left_thigh_q_z': [quat_row[0][3] for quat_row in orientations],
        'left_thigh_q_w': [quat_row[0][0] for quat_row in orientations],

        'right_thigh_q_x': [quat_row[1][1] for quat_row in orientations],
        'right_thigh_q_y': [quat_row[1][2] for quat_row in orientations],
        'right_thigh_q_z': [quat_row[1][3] for quat_row in orientations],
        'right_thigh_q_w': [quat_row[1][0] for quat_row in orientations],

        'torso_q_x': [quat_row[2][1] for quat_row in orientations],
        'torso_q_y': [quat_row[2][2] for quat_row in orientations],
        'torso_q_z': [quat_row[2][3] for quat_row in orientations],
        'torso_q_w': [quat_row[2][0] for quat_row in orientations],
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

def prepare_ik_solver(model:osim.Model, oRefs:osim.OrientationsReference,
                      constraint_var:float=10.0,
                      accuracy:float=0.001):
    
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
            print(coord.getName(), coord.getValue(state))
            ik_results.loc[i, joint_name] = coord.getValue(state)
        
    return ik_results

def df_to_orientation_refs(orientation_df:pd.DataFrame, model:osim.Model, initial_state:osim.State, 
                           zero_pose_timestamp:float, sensor_bone_offsets:dict, data_label_to_osim:dict=None
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
    col_labels = list(orientation_df.columns)
    body_names = list(set([col.split('_q_')[0] for col in col_labels if '_q_' in col]))

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
        odom_ref = R.from_quat(odom_ref).as_matrix()
        sensor_offset = sensor_bone_offsets[body_name]
        
        ref_pose[body_name] = R_GB @ sensor_offset @ odom_ref.T

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
    
            R_rel = ref_pose[body_name] @ R.from_quat(cur_quat).as_matrix()  
            quat_rel = R.from_matrix(R_rel).as_quat()  # Convert back to quaternion

            updater = rvq.updElt(0, j)
            [updater.set(k, quat_rel[k]) for k in range(4)]
            
        quatTable.appendRow(row["timestamp"], rvq)

    # convert quat time series table to rotation matrix time series table
    rotTable = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
    return osim.OrientationsReference(rotTable)

def osim_rot_to_numpy(osim_R:osim.Rotation):
    R = osim_R.asMat33()   # SimTK Mat33
    R_np = np.array([
        [R.get(0,0), R.get(0,1), R.get(0,2)],
        [R.get(1,0), R.get(1,1), R.get(1,2)],
        [R.get(2,0), R.get(2,1), R.get(2,2)],
    ])

    return R_np

if __name__ == "__main__":
    # sensor offest angles should be in the frame of the parent bone. Generally opensim has +z pointing to the right (from a first person perspective)
    #     +x point forward, and +y pointing up (feet to head)


    model_folder = "models/gait2354"  # path to folder containing model.osim and geometry
    #orientation_csv = "path/to/orientation_data.csv"

    model, state = prepare_model(model_folder)
    orientation_df = generate_sinusoid_data(num_samples=1000, 
                                            frequency=0.5, 
                                            amplitude=np.pi/4,
                                            central_point=0)  # Generate sinusoidal data with a central point of pi/4
    
    # in the data csv, I use left_arm_q_x... right_arm_q_x... torso_q_x...
    # but we can convert to opensim body names using this dict!
    data_label_to_osim = {"left_thigh": "femur_l", 
                          "right_thigh": "femur_r", 
                          "torso": "pelvis"}
    
    desired_joints = ["hip_flexion_r", "hip_flexion_l"]  # specify which joints we want to save from the IK results
    zero_pose_timestamp = 0.5  # specify the timestamp in the orientation data that corresponds to the zero pose of the model
    
    sensor_bone_offsets = {
        "left_thigh": R.from_euler('xyz', [0, 0, 0]).as_matrix(),  # No offset for left thigh
        "right_thigh": R.from_euler('xyz', [0, 0, 0]).as_matrix(),  # No offset for right thigh
        "torso": R.from_euler('xyz', [0, 0, 0]).as_matrix(),  # No offset for torso
    }

    oRefs = df_to_orientation_refs(orientation_df, model, state, 
                                   zero_pose_timestamp, sensor_bone_offsets,
                                   data_label_to_osim=data_label_to_osim)
    
    ik_solver = prepare_ik_solver(model, oRefs)

    # Perform IK and save results to CSV
    ik_results = solve_ik(model, ik_solver, state, orientation_df["timestamp"], desired_joints)

    print(ik_results)

    # wow it solves very fast, maybe track() is quicker with higher rate data? Oh and accuracy was
    # set to values from Patrick's script. Maybe we have too high of an accuracy requirement on 
    # the realtime case and could speed up tracking even more!

    #TODO: get this working on another pc, the ROG seems to have issues with the built-in osim visualizer,
    #      but this should be very easy to implement
    #model.setUseVisualizer(True)
    #visualize_ik_results(model, ik_results)
