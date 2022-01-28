#!/bin/bash -e
cd ~/catkin_ws
catkin_make
exec "$@"
