#!/bin/bash -e
gnome-terminal -x pkill aplay &
sleep 0.1
gnome-terminal -x aplay ~/sound/ssmc.wav &
exit 0
