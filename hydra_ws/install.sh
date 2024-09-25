#!/bin/bash -e

source /opt/ros/noetic/setup.bash

# Get current file location
INSTALL_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR=$INSTALL_SCRIPT_DIR/src
echo "INSTALL_SCRIPT_DIR: $INSTALL_SCRIPT_DIR"

# Catkin workspace initialization
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --skiplist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools rviz_map_plugin minkindr_python

# Import ROS dependencies
cd $SRC_DIR
vcs import . < hydra/install/hydra.rosinstall

# Install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
cd $INSTALL_SCRIPT_DIR
catkin build
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source $PWD/devel/setup.bash

# Log the success message
echo "Hydra workspace installed successfully!"