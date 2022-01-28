#!/bin/bash -e
source ~/globotix.sh
roslaunch htbot rosbagmap.launch &
rosbag play --clock ~/Downloads/globotixD.bag &
rosrun rviz rviz -d ~/.rviz/rosbagmap.rviz
