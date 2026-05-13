# Project Argus
This repo uses ros2 humble to combine data from multiple odometry sensors affixed to human body segments, and publishes the estimated biological joint angles with respect to human skeletal constraints.

This work relies on real-time modifications to opensim made for OpenSenseRT/RealTimeKin: https://github.com/pslade2/RealTimeKin .

## Offline IK Setup Instructions
For offline IK, we don't need ros or realtime opensim functionality, so we can use the opensim pip packages. The following is tested on Ubuntu 22.04.

```
conda create -n opensim_env python=3.10
conda activate opensim_env
pip install matplotlib numpy scipy pandas
conda install -c opensim-org opensim
```

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

Currently this repo supports a bimanual upper arm model which is saved in the repo (thanks to MOBL: https://simtk.org/projects/upexdyn). It has been modified from what is available online for faster IK (removed muscles, locked certain coordinates)

To expand this to other body segments (like the legs) please feel free to add to the models folder and make a PR. Sensor topic names should correspond to the bone to which they are attached in opensim, check your model file to determine this.

For setting sensor offset axes, I suggest using xyz euler angles because it's more intuitive. Imagine applying the transforms in xyz order to the osim bone axes, so that they align with the sensor axes. This is the transform from bone, to sensor.


## Bag Synchronization
This is a tool used to merge multiple bags and order the resultant bag based on the input header.stamps. Intended to help combine data collected by multiple sensors synchronized to a world clock.

For python setup:
```
python3-venv 
```