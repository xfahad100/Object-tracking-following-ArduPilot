# Object-tracking-following-ArduPilot

This repo is based on implementation of OpenCV object tracking and following the target within the field of view of the camera and the target is selected by a mouse click.
Tested on:
1. Ubuntu 20.04
2. ROS noetic
3. Gazebo 11

Demo video:
https://www.youtube.com/watch?v=hYx90dD13Wk

You may have to tune PIDs according to your vehicle in follow_target.py

First, setup ArduPilot SITL and then clone this repo into your catkin workspace.

Terminal 1:

roslaunch gz_launch_ros distance_sensor_gazebo.launch

Terminal 2:

 ../Tools/autotest/sim_vehicle.py -f gazebo-iris --out udp:127.0.0.1:14551 --console

Terminal 3:

roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@"

Terminal 4:

rosrun follow_tgt_opencv follow_target.py

