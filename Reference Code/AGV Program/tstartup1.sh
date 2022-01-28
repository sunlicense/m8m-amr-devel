#!/bin/bash -e
source ~/catkin_ws/devel/setup.bash
#source ~/nyssetup.sh
source ~/globotix.sh
~/dellog.sh
sleep 5
#roslaunch htbot htbot_nav.launch &
#roslaunch htbot htbot_map.launch &
#roslaunch htbot htbot_nav_kyu.launch &
#roslaunch htbot raclum.launch &
#roslaunch htbot htbot_map_sick.launch &
roslaunch htbot htbot_global.launch &
#roslaunch htbot test_dyna.launch &
exit "$@"
