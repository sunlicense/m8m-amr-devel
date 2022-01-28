#!/bin/bash -e
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://rac0:11311
export ROS_HOSTNAME=192.168.13.100
export ROS_IP=192.168.13.100
#export ROS_MASTER_URI=http://127.0.0.1:11311
#export ROS_HOSTNAME=127.0.0.1
#export ROS_IP=127.0.0.1
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/rac
export ROSLAUNCH_SSH_UNKNOWN=1
rosclean purge -y
sudo /etc/init.d/lighttpd stop
sleep 1
exec "$@"
