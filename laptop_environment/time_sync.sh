#!/usr/bin/env bash
echo "syncing time"
sudo /etc/init.d/chrony stop
sudo ntpdate 192.168.1.67
#sudo /etc/init.d/chrony start
sudo ntpdate -q 192.168.1.67
