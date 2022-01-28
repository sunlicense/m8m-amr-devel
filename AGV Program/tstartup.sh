#!/bin/bash -e
source ~/catkin_ws/devel/setup.bash
#source ~/nyssetup.sh
source ~/globotix.sh
~/dellog.sh
sleep 1
roslaunch htbot globotix_master.launch &
sleep 10
rosparam set startRAC1 true
sleep 2
rosparam set startRAC1 true
sleep 2
rosparam set startRAC1 true
sleep 2
rosparam set startRAC1 true
exit "$@"
