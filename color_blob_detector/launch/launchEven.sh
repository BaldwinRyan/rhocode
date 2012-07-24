#!/bin/bash

for (( i = 0 ;  i < 180;  i+=2 ))
do
    roslaunch color_blob_detector hue_monitor.launch hue:=$i window:=10 &
done
