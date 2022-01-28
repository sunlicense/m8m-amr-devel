#!/bin/bash
# date updated : 29.4.21 10am
#baseIP="10.0.5.103" ;
#ntuc=`ifconfig  | grep 'inet addr:10.0.5.102' | cut -d: -f2 | awk '{ print $1}'`;
#echo "baseIP = $baseIP. ntuc=$ntuc";
while [ 1 ] ; do
 	ntuc=`ifconfig  | grep 'inet addr:10' | cut -d: -f2 | awk '{ print $1}'`
	#echo "NTUC IP = $ntuc"
	if [ "$ntuc" != "10.0.5.102" ]; then 
		#echo "NTUC is UP"
  #else
		echo "NTUC IP is Down ...."
		sudo service network-manager restart &
	fi	
	sleep 5
done
exit
