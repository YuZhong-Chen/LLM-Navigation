Architecture
============

Software
--------

.. image:: ./images/software_arch.png
    :align: center
    :alt: Software Architecture

Our software architecture consists of four main modules. The low-level control module receives goal points and control signals to direct the robot's movements. 
The segmentation module processes RGBD images from the robot's sensors (camera, LiDAR, odometry) and identifies different objects and regions in the environment. 
The scene graph (SG) builder module constructs a 3D scene graph from the segmented data, representing spatial relationships and object attributes. 
Finally, the language query module, powered by a large language model, interprets high-level instructions and converts them into goal points for the low-level controller, 
enabling the robot to understand and execute complex tasks. Later, we will introduce our implementation details in the following paragraph.

Hardware
--------

.. image:: ./images/hardware_arch.png
    :align: center
    :alt: Hardware Architecture

In our hardware architecture, various components work together to enable the robot to navigate and interact with its environment efficiently. 
The AMD KR260 serves as the core component, interfacing with multiple sensors including the Realsense camera, VLP16 LiDAR, and the Kobuki wheeled robot. 
The KR260 processes input from these sensors, utilizing SLAM to construct a basic 2D costmap for navigation and building a 3D scene graph from labeled images. 
Due to the high computational load, a local server is employed for semantic segmentation, 
sending the labeled images back to the KR260 to build a semantic 3D scene graph. The scene graph is encoded into a hierarchical YAML file, 
which is then queried by a remote server hosting a Large Language Model (LLM) with goal descriptions such as "I want to go to bed." 
The LLM processes these queries and returns a goal point, which the KR260 uses to navigate the robot to the desired location.
