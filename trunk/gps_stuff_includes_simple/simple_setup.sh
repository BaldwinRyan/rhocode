#!/bin/bash

address=$(python gps_setup.py)

#export ROS_MASTER_URI=$address
echo "Connect to "
echo $address
stty -F /dev/ttyUSB0 clocal
roslaunch simple simple_all.launch
#roslaunch rflex atrvjr-drivers.launch &
#roslaunch simple simple_1.launch &
#roslaunch pgr_camera_driver camera_stereo_proc.launch &
#roslaunch color_blob_detector double.launch &
#rosrun color_blob_detector triangulation &
#srosrun color_blob_detector cone_present & 

#exit 0
