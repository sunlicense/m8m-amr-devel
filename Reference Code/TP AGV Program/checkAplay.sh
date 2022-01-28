#!/bin/bash
while [ 1 ] ; do
	pidof  aplay >/dev/null
	if [[ $? -eq 0 ]] ; then
		#echo "aplay is playing ..."
		rosparam set AplayRunning true
	else 
		#echo "aplay has stopped..."
		rosparam set AplayRunning false
	fi
	sleep 1
done
echo "------ checkAplay exiting -----"
exit

