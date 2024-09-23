Hydra Workspace
===============

.. note::

    This workspace is primarily adapted from Hydra. 
    For more detailed information, please refer to the `original github repository <https://github.com/MIT-SPARK/Hydra>`_ or read the `paper <https://arxiv.org/abs/2201.13360>`_.

Building docker image
----------------------

1. Clone the repository::

    git clone https://github.com/YuZhong-Chen/LLM-Navigation.git

2. Build the image::

    cd LLM-Navigation/hydra_ws/docker
    docker compose pull
    docker compose run --rm --build hydra-ws-build

Running hydra with default settings
------------------------------------

1. Run the container::

    docker compose run --rm hydra-ws

2. Run the hydra::

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch hydra_ros uhumans2.launch