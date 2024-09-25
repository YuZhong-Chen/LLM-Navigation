#!/bin/bash -e

# Running udevd in the background, it will listen for events from the kernel and manage the device nodes in /dev.
# Reference: 
# - https://forums.docker.com/t/udevadm-control-reload-rules/135564/2
# - https://manpages.ubuntu.com/manpages/focal/en/man8/systemd-udevd-kernel.socket.8.html
if ! pidof "systemd-udevd" > /dev/null; then
    echo "Launching systemd-udevd ..."
    sudo /lib/systemd/systemd-udevd --daemon &> /dev/null
fi

echo "Reloading the udev rules ..."

# Reload the udev rules.
# Reference: https://unix.stackexchange.com/a/39371
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

echo "Done."

# Source workspace environment.
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/user/realsense_ros1_ws/devel/setup.bash

# Bring up the realsense camera.
roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=480 color_fps:=15 depth_width:=640 depth_height:=480 depth_fps:=15 align_depth:=true