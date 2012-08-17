#!/bin/bash

address=$(python gps_setup.py)

export ROS_MASTER_URI=$address
echo "Connect to "
echo $address
roslaunch simple simple_1.launch
#exit 0