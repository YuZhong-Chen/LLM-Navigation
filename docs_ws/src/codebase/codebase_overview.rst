.. _codebase_overview_link:

CodeBase Overview
=================

To ensure better maintenance of our code, we have packaged all the necessary code into Docker and divided it into multiple workspaces based on functionality. 
Normally, you only need to navigate to the specified workspace location and use ``docker compose up`` to complete all environment setup.

Below is the complete list of workspaces along with a brief introduction to each workspace.

- cartographer_ws
    - Setup the Cartographer SLAM.
- gazebo_world_ws
    - Generate the Gazebo world.
- hydra_ws
    - Generate the scene graph in real-time.
- kobuki_ws
    - Bring up the kobuki robot.
- realsense_ros1_ws
    - Bring up the Intel Realsense camera.
- ros1_bridge_ws
    - Message exchanging between ROS1 noetic and ROS2 humble.
- vlp_ws
    - Bring up the VLP-16 LiDAR.
- llm_query_ws
    - Query the LLM.