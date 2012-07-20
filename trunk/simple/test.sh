#!/bin/bash
rosmake -s
rosrun simple motion_controller &
rosrun simple gps_test_input 


exit 0