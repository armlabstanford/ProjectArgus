pkill -f ros2
pkill -f rviz
pkill -f gazebo
source /opt/ros/humble/setup.bash 
ros2 daemon stop
ros2 daemon start
ros2 node list
