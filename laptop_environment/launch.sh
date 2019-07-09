#!/usr/bin/env bash
sudo ./time_sync.sh
roscore &
rviz &
rqt &
roslaunch move_base move_base.launch &
rosrun web_video_server web_video_server &
roslaunch gmapping perceptbot_gmapping.launch
#roslaunch hector_slam_launch mapping_box.launch
