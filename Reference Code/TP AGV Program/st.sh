#!/bin/bash -e
source /home/rac/catkin_ws/devel/setup.bash
#export ROS_MASTER_URI=http://127.0.0.1:11311
#export ROS_HOSTNAME=127.0.0.1
#export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://172.23.44.12:11311
export ROS_HOSTNAME=172.23.44.12
export ROS_IP=172.23.44.12
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/rac
export ROSLAUNCH_SSH_UNKNOWN=1
rosclean purge -y
#sudo /etc/init.d/lighttpd stop
sleep 1
killall checkIP.sh &
sleep 2
~/checkIP.sh &
sleep 5
roslaunch htbot ntuc_nav.launch
exec "$@"
