Quick start
===========

Before you start, make sure you have read the :ref:`codebase_overview_link` section and fully understand our architecture.
This will help you better comprehend the purpose of each step.

.. warning::

    To enhance the possibility of reproduction, we provide Dockerfile and Docker Compose to help set up the environment. 
    However, due to variations in the actual robot configurations, some Docker settings may need to be adjusted manually.

1. Clone the repository::

    git clone https://github.com/YuZhong-Chen/LLM-Navigation.git

2. Build the docker container for every workspace::

    cd LLM-Navigation

    cd cartographer_ws
    docker compose build
    cd ..

    cd gazebo_world_ws
    docker compose build
    cd ..

    cd hydra_ws
    docker compose build
    cd ..

    cd kobuki_ws
    docker compose build
    cd ..

    cd llm_query_ws
    docker compose build
    cd ..

    cd realsense_ros1_ws
    docker compose build
    cd ..

    cd vlp_ws
    docker compose build
    cd ..

    cd ros1_bridge_ws
    docker compose build
    cd ..

3. Start the ros1_bridge

Please modify the ``ROS_HOSTNAME`` and ``ROS_MASTER_URI`` in the ``ros1_bridge_ws/docker/.env`` based on your computer setup. 
Then, use ``docker compose up`` to create the ros1_bridge container and start the ros1_bridge.

4. Bring-up the Robot

Run the script ``run.sh`` in ``scripts`` folder to bring up Kobuki, VLP-16 and Realsense. 
You may need to modify some settings based on your robot configuration.

5. Generate the 3D Scene Graph with LLM Guidance

After building the package in ``hydra_ws``, use the command ``roslaunch hydra_ros run_sg.launch`` to generate the 3D scene graph. 
Then, use the ``query.py`` script in ``llm_query_ws`` to query the LLM with the current scene graph.