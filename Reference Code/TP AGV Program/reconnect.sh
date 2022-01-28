#!/bin/bash -e
#aplay ~/sound/RadioOff.wav &
ssid="AGV-130"
nmcli r wifi off &
sleep 1
#aplay ~/sound/RadioOn.wav &
nmcli r wifi on &
sleep 1
#aplay ~/sound/ConnectAP.wav &
#sleep 3
nmcli device wifi connect $ssid password AutomaticRobot!130
#nmcli device wifi connect WLAN_MFG password MFG@Main190y$n 
#nmcli device wifi connect racwifi password racwifi777
#sleep 1
#exit 0
