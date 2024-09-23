Kobuki Workspace
================

This repository facilitates the quick configuration of the simulation environment and real robot driver for Kobuki.

Building docker image
----------------

1. Clone the repository::

    git clone https://github.com/YuZhong-Chen/LLM-Navigation.git

2. Build the docker image::

    cd LLM-Navigation/kobuki_ws/docker
    docker compose pull
    docker compose up -d --build

Building the workspace
----------------

1. Attach to the container::

    docker attach ros2-kobuki-ws

2. Compile the workspace::

    # For x86_64 architecture
    colcon build --symlink-install
    # For arm64 architecture
    colcon build --symlink-install --packages-ignore velodyne_gazebo_plugins

Simple test in Gazebo
----------------

1. Attach to the container::

    docker attach ros2-kobuki-ws

2. Launch the world::

    ros2 launch kobuki_launch kobuki.launch.py is_sim:=true

Simple test on real robot
----------------

1. Attach to the container::

    docker attach ros2-kobuki-ws

2. Bringup script for real robot (make sure kobuki is connected to the computer)::

    cd /home/ros2-essentials/kobuki_ws
    ./script/kobuki-bringup.sh

3. Running the script for controlling the robot::

    cd /home/ros2-essentials/kobuki_ws
    ./script/kobuki-teleop.sh


.. note::
    For more details, please refer to the ``README.md`` file in the repository.