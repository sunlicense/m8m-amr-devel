#!/bin/bash -e
ps aux | grep -i go.sh | awk {'print $2'} | xargs kill -9 &
sleep 0.2
ps aux | grep -i aplay | awk {'print $2'} | xargs kill -9 &
exit
