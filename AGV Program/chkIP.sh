#!/bin/bash
# date updated : 29.4.21 10am
source /home/rac/catkin_ws/devel/setup.bash
#export ROS_MASTER_URI=http://127.0.0.1:11311
#export ROS_HOSTNAME=127.0.0.1
#export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://10.0.5.102:11311
export ROS_HOSTNAME=10.0.5.102
export ROS_IP=10.0.5.102
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/rac
export ROSLAUNCH_SSH_UNKNOWN=1
while [ 1 ] ; do	
 	ntuc=`ifconfig  | grep 'inet addr:10' | cut -d: -f2 | awk '{ print $1}'`
	rosparam set Network false
	if [ "$ntuc" == "10.0.5.102" ]; then 
		rosparam set Network true
		echo "Network OK...."
  else
		#rosparam set Network false
		echo "Network Down...."
	fi	
	sleep 1
done
exit
