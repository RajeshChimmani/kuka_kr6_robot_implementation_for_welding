#!/bin/bash
unset LD_LIBRARY_PATH QT_PLUGIN_PATH LD_PRELOAD
source "/opt/ros/jazzy/setup.bash"
ros2 launch "/home/rajesh/implementation_of_kuka/custom_kuka_description/launch/gazebo.launch.py" play:=true
