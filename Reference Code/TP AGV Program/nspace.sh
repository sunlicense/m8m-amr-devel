#!/bin/bash -e
#gnome-terminal -x ~/ssmcend.sh &
gnome-terminal -x ~/ssmcend.sh &
sleep 0.2
#gnome-terminal -x aplay -D plughw:1 ~/sound/needspace.wav &
gnome-terminal -x aplay ~/sound/needspace.wav &
sleep 2
gnome-terminal -x pkill aplay &
#pkill aplay
sleep 0.2
#gnome-terminal -x ~/ssmc.sh &
gnome-terminal -x ~/ssmc.sh &
exit 0
