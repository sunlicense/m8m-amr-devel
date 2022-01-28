#!/bin/bash -e
sudo rm -rf /home/rac/Downloads/rac0Backup/
sudo rm -rf /home/rac/Downloads/rac1Backup/
sleep 1
mkdir /home/rac/Downloads/rac0Backup
mkdir /home/rac/Downloads/rac1Backup
sshpass -p "rac" scp -r rac@rac1:/home/rac/catkin_ws/src  /home/rac/Downloads/rac1Backup/ &
#sleep 15
sshpass -p "rac" scp -r rac@rac1:/home/rac/www  /home/rac/Downloads/rac1Backup/ &
sshpass -p "rac" scp -r rac@rac1:/home/rac/*.sh  /home/rac/Downloads/rac1Backup/ &
cp -R /home/rac/catkin_ws/src  /home/rac/Downloads/rac0Backup/ &
cp /home/rac/*.sh  /home/rac/Downloads/rac0Backup/ &
cp -R /home/rac/sound  /home/rac/Downloads/rac0Backup/ &
sleep 5
