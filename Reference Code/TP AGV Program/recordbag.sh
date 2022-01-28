#!/bin/bash -e
# /frontHD/image_raw/compressed
# /odom
# /rearHD/image_raw/compressed
# /rightRS/color/image_raw/compressed
# /leftRS/color/image_raw/compressed
# /robot_pose
# /scanF
# /tf
# /rightRS/imageRQ/compressed
# /rearHD/imageRQ/compressed
# /leftRS/imageRQ/compressed
# /frontHD/imageRQ/compressed
#rosbag record -O ~/Downloads/globotix.bag /tf /scanF /scanC /scanR /frontHD/image_raw/compressed /rearHD/image_raw/compressed /leftRS/color/image_raw/compressed /rightRS/color/image_raw/compressed /odom /robot_pose __name:=record
# rosbag record -O ~/Downloads/globotix.bag /tf /scanF /odom /robot_pose /frontHD/image_raw/compressed /rearHD/image_raw/compressed /leftRS/color/image_raw/compressed /rightRS/color/image_raw/compressed  __name:=record
rosbag record -O ~/Downloads/globotix.bag /tf /scanF /odom /robot_pose /frontHD/imageRQ/compressed /rearHD/imageRQ/compressed /rightRS/imageRQ/compressed /leftRS/imageRQ/compressed __name:=record
#rosbag record -O ~/Downloads/globotix.bag /tf /scanF /odom /robot_pose /frontHD/imageRQ /rearHD/imageRQ /rightRS/imageRQ /leftRS/imageRQ __name:=record
