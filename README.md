# Project Argus
This repo uses ros2 humble to combine data from multiple odometry sensors affixed to human body segments, and publishes the estimated biological joint angles with respect to human skeletal constraints.

This work relies on real-time modifications to opensim made for OpenSenseRT/RealTimeKin: https://github.com/pslade2/RealTimeKin .

## What's in git
Git tracks what you need to **run the system live**. Session bags, calibrations, mocap, plots, and offline analysis stay on disk (see `.gitignore`).

| Path | Role |
|---|---|
| `src/argus_server/` | Realtime OpenSim IK node, RViz launch, URDFs |
| `src/argus_drift/` | Vision drift correction (all camera pairs, full SO(3)) |
| `src/argus_mahoney/` | IMU-only Mahony orientation node |
| `cam/`, `imu/` | Pi camera and BMI270 nodes |
| `models/` | OpenSim models, `argus_model.json`, geometry |
| `argus_gui.py`, `skeleton_view.py` | Handheld GUI and skeleton view |
| `scripts/` | Network, module bringup, recording, OpenVINS/Mahony tracking (`track.sh`, `sync_tracked.sh`, `record_joints.sh`) |
| `argus.conf`, `source_env.sh` | Module network names; ROS 2 + OpenSim env |
| `poses/poses.json`, `poses/rajagopal_upper/poses.json`, `poses/images/` | Upper-limb zero postures and their photos |
| `calibration.md` | IMU / camera (Kalibr) workflow notes |

Not in git (local only): `poses/*/calibrations/`, captures, `argus_wide_calib/`, `comparisons/`, `c3d_data/`, `activity_tasks*.json`, `mocap_ik/`, plot/render/compare scripts, `pose_capture.py`, `estimate_mounts.py`, `drift_offline.py`, `mahony_offline.py`, and `scripts/tracking/replay_joints.sh` / `bag_health.sh` / `restamp_tracked.py`. Default sensor mounts live in each model's `argus_model.json`; the newest `calibration_*.json` in that model's `calibration_dir` replaces them at runtime if present.

## OpenSim-ROS2 Setup Instructions
I experienced issues when building opensim with an active ros environment due to fmt versions. I would advise uninstalling ros2 temporarily, then performing the following steps to build opensim from source:

1) uninstall ros2-humble-desktop and specifically libfmt-dev and libspdlog-dev or any variants
2) then, install opensim from source (without using their automated setup!) followed instructions here: https://github.com/opensim-org/opensim-core/wiki/Build-Instructions-(Old-deprecated). install components and do not use system deps
3) use cmake to install to ~/opensim-core (outlined in tutorial)
4) install ros-humble-desktop deb packages through instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
5) now, set env variables. unsure if all of these are strictly needed, but the sum total worked for me:
```
source /opt/ros/humble/setup.bash
export OpenSim_DIR=~/opensim-core
export LD_LIBRARY_PATH=/home/argus/opensim-core/lib:$LD_LIBRARY_PATH
```
LD_LIBRARY_PATH is how you can find opensim .so files at runtime
OpenSim_DIR I believe is used in the build itself
6) from the root of this repo, 
```
colcon build
```
If build fails saying things about cmake and ament, you forgot to source ros2.
If build says it can't find opensim_dir or the package, either the setup didn't work, or you just need to set OpenSim_DIR as explained above
7) now source what you just built:
``` 
source install/setup.bash
```
8) test out the node launch:
```
ros2 launch argus_server argus_server.launch.py
```

Please report any issues with setup to improve these steps! I messed with this for a while so there could be things that ought to be included on some systems.

For issues regarding missing .so files, execute the following:
```
export LD_LIBRARY_PATH=/home/argus/opensim-core/lib:$LD_LIBRARY_PATH
```
You could also add this to your ~/.bashrc

## Usage Instructions

Bringup the argus server:
```
ros2 launch argus_server argus_server.launch.py
```

And visualization via rviz:
```
ros2 launch argus_server human_model.launch.py
```

for testing:
```
ros2 topic pub --once /thorax/odomimu nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 0.5, y: 0.2, z: 0.1}, orientation: {w: 1.0}}}}"
ros2 topic pub --once /humerus_l/odomimu nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 0.5, y: 0.2, z: 0.1}, orientation: {w: 1.0}}}}"
ros2 topic pub --once /reset_tracking std_msgs/msg/Bool "{data: true}"
```

Then:
```
ros2 launch argus_server fake_data.launch.py
```

The default live model is `models/rajagopal_upper/` (Rajagopal 2015, unused joints locked). `models/rajagopal_lower/`, `models/bimanual_upper_arm/` (MOBL Holzbaur), and `models/gait2354/` are also in the repo. Sensor topic names should correspond to the bone to which they are attached in OpenSim (`argus_model.json`).

Default sensor-to-bone mounts are xyz Euler degrees in `argus_model.json` (`body_calibrations`): apply the transforms in xyz order to the OpenSim bone axes so they align with the sensor axes. Session calibrations are not committed.

Handheld GUI (modules, model, drift fix, calibrate):
```
python3 argus_gui.py
```

## Shell Scripts
The shell scripts live in `scripts/` (`source_env.sh` stays at the repo root):
```
scripts/
  all_active.sh      run a module script on every module at once
  argus_common.sh    shared module lookup (sourced; reads argus.conf from the repo root)
  super_kill_ros.sh
  network/           hotspot, module status, SSH keys, Wi-Fi networks on the modules
  module/            per-Pi bringup (sensors, OpenVINS, Mahony), updates, Kalibr load
  tracking/          track.sh, sync_tracked.sh, record_joints.sh
```
They can be run from any directory. All scripts run on the laptop unless noted. Module names are `argus0`..`argus4`; scripts find each module's address from the hotspot leases (or by scanning the attached network) and reach it over SSH as `argus` (passwordless SSH set up with `scripts/network/add_known_host.sh`). Network names, interfaces, and the SSH user come from `argus.conf` (local overrides in `argus.local.conf`, which is not committed).

### Setup and network
| Script | What it does | Example |
|---|---|---|
| `source_env.sh` | Environment for everything here: ROS 2 Humble, OpenSim, this workspace. Source it, from any directory. | `source source_env.sh` |
| `scripts/argus_common.sh` | Shared module lookup (names, addresses, SSH) used by the other scripts. Sourced, not run. | |
| `scripts/network/start_hotspot.sh` | Hosts the `argus` access point on the Archer USB adapter (plain WPA2) and shares the built-in radio's internet with the modules (NAT). Needs sudo. | `sudo scripts/network/start_hotspot.sh` |
| `scripts/network/argus_status.sh` | Pings every module and shows which network they are on. | `scripts/network/argus_status.sh` |
| `scripts/network/add_known_host.sh` | Installs this machine's SSH key on a device, for passwordless SSH. | `scripts/network/add_known_host.sh 10.42.0.37 ArmLab19 argus` |
| `scripts/network/wifi_net.sh` | Adds (as preferred, keeping the old ones as fallback) or removes a Wi-Fi network on every module that answers. | `scripts/network/wifi_net.sh add ARMLab-5G PASSWORD`<br>`scripts/network/wifi_net.sh remove OldSSID` |
| `scripts/network/fix_pi_sdcard.sh` | Clears a persistent firewall on a Pi SD card mounted on this machine, so SSH answers again. | `sudo scripts/network/fix_pi_sdcard.sh /mnt/pi` |
| `scripts/super_kill_ros.sh` | Kills local ros2 / rviz / gazebo processes and restarts the ROS daemon. | `scripts/super_kill_ros.sh` |

### One module
Each runs in the foreground over SSH; Ctrl-C stops the remote nodes.

| Script | What it does | Example |
|---|---|---|
| `scripts/module/bringup_sensors.sh` | Starts the camera, IMU, and status LED nodes on a Pi. | `scripts/module/bringup_sensors.sh argus2` |
| `scripts/module/bringup_openvins.sh` | Starts OpenVINS on a Pi's camera and IMU, plus the 50 Hz relay to `/<module>/odomimu_throttled` for the IK server. | `scripts/module/bringup_openvins.sh argus2` |
| `scripts/module/bringup_mahoney.sh` | IMU-only Mahony orientation, published on `/<module>/mahoney_odomimu`. Hold the module still for the first 10 s (gyroscope bias). | `scripts/module/bringup_mahoney.sh argus2` |
| `scripts/module/pull_module.sh` | `git pull` (fast-forward only) and Release build of `~/ArgusModule` on a Pi. | `scripts/module/pull_module.sh argus2` |
| `scripts/module/load_calib.sh` | Merges a Kalibr result folder into the OpenVINS calibration on the matching module(s). | `scripts/module/load_calib.sh argus_wide_calib`<br>`scripts/module/load_calib.sh argus_wide_calib argus3` |

### All modules at once
`scripts/all_active.sh` runs one of the module scripts on every module that answers ping, side by side, with each line prefixed by the module name. Ctrl-C stops them all.

| Command | Runs on each module |
|---|---|
| `scripts/all_active.sh bringup` | `scripts/module/bringup_sensors.sh` |
| `scripts/all_active.sh openvins` | `scripts/module/bringup_openvins.sh` |
| `scripts/all_active.sh mahoney` | `scripts/module/bringup_mahoney.sh` |
| `scripts/all_active.sh pull` | `scripts/module/pull_module.sh` |

`all_active.sh` also has record / play / fetch / cook dispatchers; those call helpers under `scripts/bags/`, which are not in git.

### Tracking
Bags live in `~/bags/` on this machine and on each Pi (not in git).

| Script | What it does | Example |
|---|---|---|
| `scripts/tracking/track.sh` | Opens three terminals: plays the bag on every module, runs OpenVINS (or `--mahoney`) on every module, records the output here. When the recording is stopped with Ctrl-C it is re-timed with `scripts/tracking/sync_tracked.sh`. Writes `~/bags/<bag>_tracked` (or `<bag>_mahoney_tracked`). | `scripts/tracking/track.sh trial_3_cooked`<br>`scripts/tracking/track.sh --mahoney trial_3_cooked` |
| `scripts/tracking/sync_tracked.sh` | Re-times a tracked bag to the module sample stamps (each module's playback starts at a slightly different moment); the original is kept as `*.unsynced`. No argument: every `~/bags/*_tracked`. | `scripts/tracking/sync_tracked.sh ~/bags/trial_3_cooked_tracked` |
| `scripts/tracking/record_joints.sh` | Records the server's `/joint_states` (and resets / zero pose loads) here; `--poses` also records the module poses. Does not start the server. | `scripts/tracking/record_joints.sh dynamic1` |

Live collection:
```
sudo scripts/network/start_hotspot.sh                  # once, in its own terminal
scripts/all_active.sh bringup                  # sensors on every module
python3 argus_gui.py                           # Start model, Initialize
scripts/tracking/record_joints.sh trial_3      # ~/bags/trial_3_joints; Ctrl-C to stop
```

The server loads the `neutral` pose from the model's `poses_file` (`poses/rajagopal_upper/poses.json` by default, or `poses/poses.json` for `bimanual_upper_arm`). If `calibration_dir` has any `calibration_*.json` files locally, it uses the newest; otherwise it uses `body_calibrations` in `argus_model.json`. Initialize is sent as soon as every module has delivered a pose.

## Vision Drift Correction (optional)
Each module tracks its orientation in its own world frame, and those frames slowly turn relative to each other (heading drift), which shows up as creeping joint angles. The `argus_drift` node (`src/argus_drift`) uses the module cameras to measure that drift and sends a correction to the server. It works with OpenVINS or Mahony (OpenSenseRT) tracking. The server runs exactly as before without it, and no correction is applied until the node sends one.

Start it with the server, on its own, or from the GUI (OpenSim row, `drift fix: on`, then Start model):
```
ros2 launch argus_server argus_server.launch.py drift_correction:=true                  # period 10 s
ros2 launch argus_server argus_server.launch.py model:=rajagopal_lower pose_source:=mahoney drift_correction:=true drift_period_s:=5
ros2 launch argus_drift argus_drift.launch.py model:=rajagopal_upper debug_dir:=~/drift_debug   # separately
```

What it does every `period_s` (10 s):
1. Waits (up to 8 s) for a moment when every module turns slower than 30 deg/s. Then it subscribes to `/argusN/cam0/image_raw` for up to 1 s (raw frames are large over WiFi, so the cameras are not streamed all the time) and picks one frame per module, as close together in header time as possible.
2. Takes each module's tracked orientation at its frame's header stamp, plus the Kalibr camera-IMU time shift. Modules turning faster than 90 deg/s are skipped (blur).
3. For every pair of cameras, re-renders both fisheye images into one undistorted view, placed where the server believes the two cameras look. It then matches ORB features only near where the belief puts them. Without this, the lens distortion and the very different camera rolls (the thorax camera is sideways) defeat ORB. The relative orientation is fitted from the original camera bearings: an essential matrix, or a pure rotation when the scene is too far for parallax. A pair counts if it has at least 40 inlier matches, at least 35% of its matches are inliers, and it agrees with the server's belief to within 30 deg.
4. Solves one rotation per module that makes the server's belief match all accepted pairs. The module on the model's root body (thorax or pelvis) is held fixed, and pairs inconsistent with the rest are dropped. When an arm's cameras link only to each other, that group is solved with its highest-weight module (the upper arm) held fixed. That corrects the forearm relative to the upper arm, but not the arm's heading.
5. The first three captures after each Initialize are only a baseline: whatever the mounts and zero pose leave inconsistent between cameras is not drift. Baselines are kept per module and per fixed module. After that, the change since the baseline is the drift. A share of the full SO(3) residual (gain 0.7, at most 180 deg) is sent to the server on `/argus/drift_correction`. Live correction always uses all camera pairs and full SO(3); it is not heading-only.

The server applies the correction as `R_ground_bone = W * R_ref * R_cur * C`. It publishes its zero reference and current corrections, latched, on `/argus/drift_reference`, and drops all corrections at the next Initialize. Corrections made for an older Initialize are ignored. `/argus_debug` lists the corrections in use, and `/argus/drift_debug` has the node's per-capture summary. With `debug_dir` set, the node also saves the inlier matches of every pair (`<cycle>_<a>_<b>.jpg`) and a `drift_log.jsonl`. Settings are in `src/argus_drift/config/argus_drift.yaml` (camera-IMU yaml from `argus_wide_calib/` on disk, not in git; image topic; `image_transport: compressed` if the modules publish compressed images; thresholds; gain).

Only cameras that see the same scene can be linked, so a module whose camera never overlaps another's (or links back to the root module through others) is not corrected. The node says so in its log.

## Sensor mounts
Default C matrices are `body_calibrations` in `models/<model>/argus_model.json`. The GUI Calibrate button writes a session file into that model's `calibration_dir` (local, not committed). IMU–camera Kalibr notes are in `calibration.md`; `scripts/module/load_calib.sh` copies a local Kalibr folder onto the Pis.

## Models (Rajagopal full body)
Everything model-specific lives in `models/<model>/`: the OpenSim model (`model.osim`) and `argus_model.json` (modules, default mounts and weights, published joints, `poses_file`, `calibration_dir`, URDF). The IK server and GUI use that folder.

The default is the Rajagopal 2015 full-body model (`models/rajagopal/model.osim`), with two experiment recipes that freeze unused joints so they are not solved in IK:

| | `rajagopal_upper` (default) | `rajagopal_lower` |
|---|---|---|
| Modules | argus0 torso, argus4 / argus3 right / left humerus, argus2 / argus1 right / left radius | argus0 pelvis, argus4 / argus3 right / left femur, argus2 / argus1 right / left tibia |
| Free | pelvis (root), shoulder (`arm_flex`, `arm_add`, `arm_rot`), elbow, pronation | pelvis, hip flexion / adduction / rotation, knee; ankle and lumbar if those clusters exist |
| Locked | hips, knees, ankles, lumbar, wrists | arms, wrists, subtalar, toes, lumbar (until a back plate) |
| Zero poses (in git) | `poses/rajagopal_upper/poses.json` | not committed (`poses/rajagopal_lower/poses.json` locally) |
| Session calibrations | local `poses/rajagopal_upper/calibrations/` | local `poses/rajagopal_lower/calibrations/` |
| URDF | `human_rajagopal_upper.urdf` | `human_gait.urdf` |

`bimanual_upper_arm` (Holzbaur) uses `poses/poses.json` and `poses/images/`. `gait2354` is in `models/` for old recordings. Pick the model with `model:=` (launch files) or the GUI OpenSim row (`model:`). Start model, Calibrate and the skeleton view follow it; it only switches while the model and calibration are stopped:

```
ros2 launch argus_server argus_server.launch.py model:=rajagopal_lower
ros2 launch argus_server human_model.launch.py model:=rajagopal_lower
```

The gait URDF is generated from `human.urdf` (legs renamed to the gait2354 coordinates with the model's axes, arms and spine fixed); re-run after editing either:
```
python3 models/gait2354/make_urdf.py      # writes src/argus_server/urdf/human_gait.urdf
```
