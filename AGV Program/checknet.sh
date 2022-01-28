#!/bin/bash
while [ 1 ] ; do
 	wlan=`ifconfig wlan0 | grep inet\ addr | wc -l`

	if [ $wlan -eq 0 ]; then
		nmcli d connect wlan0
	else
		echo interface is up
	fi
	sleep 10
done
exit
