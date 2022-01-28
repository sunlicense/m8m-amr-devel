#!/bin/bash
# date updated : 29.4.21 10am
#baseIP="10.0.5.103" ;
#ntuc=`ifconfig  | grep 'inet addr:10.0.5.102' | cut -d: -f2 | awk '{ print $1}'`;
#echo "baseIP = $baseIP. ntuc=$ntuc";
while [ 1 ] ; do
 	ntuc=`ifconfig  | grep 'inet addr:10' | cut -d: -f2 | awk '{ print $1}'`;
	echo "NTUC IP = $ntuc"
	if [ "$ntuc" == "10.0.5.102" ]; then 
		echo "NTUC is UP";    
  else
		echo "NTUC IP is Down ...."
		nmcli r wifi off;
		sleep 1;
		nmcli r wifi on;
		sleep 1;
		aplay ~/sound/ConnectAP.wav &
		nmcli con up 'AGV-130';
		sleep 1;
	fi
	sleep 10
done
exit
