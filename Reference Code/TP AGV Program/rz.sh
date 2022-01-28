#!/bin/bash -e
source ~/ntuc.sh
rosrun rviz rviz
exec "$@"
