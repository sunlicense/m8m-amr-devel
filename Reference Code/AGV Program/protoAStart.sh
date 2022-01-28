#!/bin/bash -e
#source ~/catkin_ws/devel/setup.bash
ulimit -n 8192 
source ~/globotix.sh
~/dellog.sh
sleep 5
roslaunch realsense2_camera rs_camera_middle.launch &
sleep 8
roslaunch realsense2_camera rs_camera_bottom.launch &
sleep 8
roslaunch realsense2_camera rs_camera_bottomR.launch &
sleep 8
roslaunch realsense2_camera rs_camera_left.launch &
sleep 8
roslaunch realsense2_camera rs_camera_right.launch &
sleep 2
roslaunch htbot globotix_master_testA.launch
#roslaunch htbot protoAMapping.launch  
#sleep 7
#roslaunch htbot combineSick.launch &
#rosparam set startRAC1 true
#sleep 2
#rosparam set startRAC1 true
#sleep 2
#rosparam set startRAC1 true
#sleep 2
#rosparam set startRAC1 true
exit "$@"
