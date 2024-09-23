#!/bin/bash -e

. activate habitat; source $ROS_WS/devel/setup.bash; cd /habitat-lab;
# roscore &
rosrun sim_test test.py
# pkill -f roscore