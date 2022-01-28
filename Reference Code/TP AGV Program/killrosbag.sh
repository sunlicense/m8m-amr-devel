#!/bin/bash -e
# /frontHD/image_raw/compressed
# /odom
# /rearHD/image_raw/compressed
# /rightRS/color/image_raw/compressed
# /leftRS/color/image_raw/compressed
# /robot_pose
# /scanF
# /tf
sleep 2
rosnode kill /record
