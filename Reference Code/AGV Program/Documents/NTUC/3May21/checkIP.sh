#!/bin/bash
# date updated : 29.4.21 10am
while [ 1 ] ; do
 	ntuc=`ifconfig  | grep 'inet addr:10'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
	echo "NTUC IP = $ntuc"
#	if [ "$lumileds" = "10.80.106.129" ]; then
	if [ "$ntuc" = "10.0.5.102" ]; then
		echo NTUC IP is Available ....
	else
		echo NTUC IP is Down ....
		nmcli r wifi off 
		sleep 1
		nmcli r wifi on 
		sleep 1
#		nmcli device wifi connect AGV-130 name AGV-130 password 'AutomaticRobot!130'
		nmcli con up 'WLAN_MFG'
	fi
	sleep 5
done
exit
