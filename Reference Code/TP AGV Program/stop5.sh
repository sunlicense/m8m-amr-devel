#!/bin/bash -e
#aplay ~/sound/Logoff.wav
sleep 5
~/ssmcend.sh
sleep 2
killall roslaunch
