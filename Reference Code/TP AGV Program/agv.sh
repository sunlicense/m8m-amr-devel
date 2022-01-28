#!/bin/bash
while [ 1 ] ; do
 aplay ~/sound/AGVMoving.wav
 #aplay -D plughw:0 ~/sound/ssmc.wav
 sleep 1
done
exit
