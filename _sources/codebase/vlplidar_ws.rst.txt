VLP-16 LiDAR Workspace
=======================

This repository contains the codebase for setting up the VLP-16 LiDAR Workspace.

Building docker image
-----------------------

1. Clone the repository::

    git clone https://github.com/YuZhong-Chen/LLM-Navigation.git

2. Build the docker image::

    cd LLM-Navigation/vlp_ws/docker
    docker compose pull
    docker compose up -d --build

Test in simulation environment
-------------------------------

1. Attach to the container::

    docker attach ros2-vlp-ws

2. Launch the world::

    ros2 launch velodyne_description example.launch.py

3. Launch LiDAR driver::

    ros2 launch vlp_cartographer vlp_driver.launch.py is_sim:=True

Test on real robot
-----------------------

1. LiDAR driver Bringup script::

    docker exec -it ros2-vlp-ws /home/ros2-essentials/vlp_ws/scripts/lidar-driver-bringup.sh

2. LiDAR SLAM test bringup script::

    docker exec -it ros2-vlp-ws /home/ros2-essentials/vlp_ws/scripts/lidar-slam-bringup.sh

.. note::
    For more details, please refer to the ``README.md`` file in the repository.