#!/bin/bash -e
source ~/catkin_ws/devel/setup.bash
source ~/nyssetup.sh
rosnode kill $(rosnode list | grep -i "map_server") &
sleep 1
roslaunch htbot htbot_mapserver.launch &
#rosrun map_server map_server /home/odroid/catkin_ws/src/htbot/maps/docmap.yaml &
exit 0
