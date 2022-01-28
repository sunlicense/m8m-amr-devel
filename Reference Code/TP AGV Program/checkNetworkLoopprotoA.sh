#!/bin/bash
count=0
countx=0
countx2=0
while [ 1 ] ; do
 	wlan=`ifconfig eth0 | grep inet\ addr | wc -l`
	if [ $wlan -eq 0 ]; then
		#rosparam set networkdown true
    nmcli c up id rv55
		echo connecting to rv55 ....
	else
		echo rv55 is up...
		##aplay ~/sound/honk.wav
		#rosparam set networkdown false
		if [ $countx -eq 1 ]; then
			aplay ~/sound/honk.wav			
			~/protoAStart.sh
		else
			echo connected ..
			#aplay ~/sound/honk.wav
		fi
		countx=$((countx+1))
	fi
	sleep 5
done
exit
