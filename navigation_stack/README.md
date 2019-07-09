ROS Navigation Stack
====================

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__navigation__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__navigation__ubuntu_bionic_amd64__binary/)

Related stacks:

 * http://github.com/ros-planning/navigation_msgs (new in Jade+)
 * http://github.com/ros-planning/navigation_tutorials
 * http://github.com/ros-planning/navigation_experimental

For discussion, please check out the
https://groups.google.com/group/ros-sig-navigation mailing list.

Initial parameter values for `move_base/params/*.yaml` were set from the following guide and
fine-tuned using trial-and-error experiments:
http://kaiyuzheng.me/documents/navguide.pdf
