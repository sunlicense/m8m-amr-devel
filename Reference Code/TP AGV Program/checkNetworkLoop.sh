#!/bin/bash
count=0
countx1=0
countx2=0
while [ 1 ] ; do
 	wlan=`ifconfig wlp2s0 | grep inet\ addr | wc -l`
	wlan1=`ifconfig wlx000db002d47c | grep inet\ addr | wc -l`
	echo "Good Counter = $count. Bad Counter1 = $countx1. Bad Counter2 = $countx2"
	if [ $wlan -eq 0 ]; then
		countx1=$((countx1+1))
		#rosparam set networkdown true
    nmcli c up id YSHome
		echo connecting to YSHome....
	else
		echo YSHome is up...
		#rosparam set networkdown false
	fi
	if [ $wlan1 -eq 0 ]; then
		countx2=$((countx2+1))
    nmcli c up id YSHome1
		echo connecting to YSHome1....
	else
		echo YSHome1 is up...
	fi
	sleep 5
	count=$((count+1))
done
exit
