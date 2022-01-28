#!/bin/bash -e
source ~/catkin_ws/devel/setup.bash
#source ~/nyssetup.sh
source ~/globotix.sh
~/dellog.sh
sleep 2
#sudo /home/nvidia/jetson_clocks.sh
#sudo nvpmodel -m 0
#sleep 2
roslaunch htbot htbot_global.launch &
#roslaunch htbot htbot_nav_kyu.launch &
#roslaunch htbot raclum.launch &
#roslaunch htbot htbot_map_sick.launch &
#roslaunch htbot htbot_map_kyu.launch &
#roslaunch htbot htbot_start.launch &
#roslaunch htbot htbot_service.launch &
#sleep 2
#~/checknet.sh &
exit "$@"
