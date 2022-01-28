#!/bin/bash -e
roslaunch videocontrol rac1_globotix.launch &
#sleep 2
roslaunch realsense2_camera rs_camera_middle.launch &
#sleep 5
roslaunch realsense2_camera rs_camera_bottom.launch &
#sleep 5
roslaunch realsense2_camera rs_camera_left.launch &
#sleep 5
roslaunch realsense2_camera rs_camera_right.launch &
#sleep 5
roslaunch videocontrol ptcloudd435_globotix_rac1.launch &
#sleep 2
roslaunch videocontrol recordvid_globotix_rac1.launch 
#nmcli device set wlx000db002d47c autoconnect yes
#nmcli device set wlp1s0 autoconnect yes
exec "$@"
