#!/bin/bash -e
rosclean purge -y
sudo rm /var/log/syslog &
sudo rm /var/log/syslog.* &
sudo rm /var/log/kern.log &
sudo rm /var/log/kern.log.* &
sudo rm /var/log/dpkg.log &
sudo rm /var/log/dpkg.log.* &
sudo rm /var/log/apport.log.* &
sudo rm /var/log/alternatives.log.* &
sudo rm /var/log/uvcdynctrl-udev.log &
exit "$@"
