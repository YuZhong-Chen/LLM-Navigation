#!/bin/bash

cd $ROS_WS

. activate habitat; pip install rospkg
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="sim_test"