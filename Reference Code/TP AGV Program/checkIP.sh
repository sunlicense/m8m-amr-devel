#!/bin/bash
# date updated : 29.4.21 10am
#baseIP="10.0.5.103" ;
#ntuc=`ifconfig  | grep 'inet addr:10.0.5.102' | cut -d: -f2 | awk '{ print $1}'`;
#echo "baseIP = $baseIP. ntuc=$ntuc";
while [ 1 ] ; do	
 	ntuc=`ifconfig  | grep 'inet addr:172' | cut -d: -f2 | awk '{ print $1}'`
	#echo "NTUC IP = $ntuc"
	if [ "$ntuc" == "172.23.44.12" ]; then 
		rosparam set 'Network' true
  else
		#echo "NTUC IP is Down ...."
		rosparam set 'Network' false
		nmcli r wifi off
		sleep 1
		nmcli r wifi on
		sleep 1
		aplay ~/sound/ConnectAP.wav
		nmcli con up 'IOT-INT'
		sleep 3
		ps aux | grep -i aplay | awk {'print $2'} | xargs kill -9 &
	fi	
	sleep 5
done
exit
