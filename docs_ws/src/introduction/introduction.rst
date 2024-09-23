Introduction
============

Introduction
------------

Navigating in a previously unseen environment is a challenging task within the field of robotic learning, 
as the agent has to perform localization, mapping and path proposing in real-time. Further, 
providing an intuitive interface for the users to guide the agent, such as natural language instructions, 
can make the task more difficult. However, it is a valuable area of study. In this research, 
the agent is tasked to navigate to the specified goal location using its forward-facing RGBD image observations and a goal query provided in natural language.

.. figure:: ./images/image_1.png
    :align: center
    :alt: image_1

    Pipeline
    
Research in 3D scene graphs (3DSG) offers valuable insights for our mapping approach. 
3DSG can hierarchically organize multiple levels of semantic meanings as nodes, including floors, places, and objects. 
The edges in a 3DSG can represent relationships between objects and the traversability between locations. 
Additionally, the real-time 3DSG construction approach presented by Hydra suggests the feasibility of integrating 3D scene graphs into our navigation task.

This detailed semantic structure can serve as a mental map for the agent, facilitating query-to-point reasoning. 
For example, given a goal query like “I want to go to sleep, where should I go?” and the current 3D scene graph, 
the LLM can guide the agent towards the appropriate point.

There are 3 main components in our architecture: 

- The AMD KR260 acts as a core component that processes the inputs from several sensors, and updating the 3D scene graph. 
- A local server response for semantic segmentation module. 
- A LLM to reason the 3DSG based on the given query, and output the location the agent should go to.

Compared to Hydra, our approach supports processing language queries, different from traditional navigation pipeline, 
which should input a specific goal by user, our pipeline significantly enhancing the flexibility of the user interface. 
Moreover, Hydra's semantic segmentation method will limit its scene understanding capabilities across diverse environments.

Initially, we aimed to address our navigation task using Reinforcement Learning (RL) methods 
due to their proven performance in obstacle avoidance and their ability to learn from various inputs 
(in our case, RGBD images, natural language, and the 3DSG). However, we found that RL methods are unstable during training, 
largely due to the sparsity of reward signals, and the noisy nature of real-world observations. Consequently, 
we transitioned from RL methods to 3DSG-based methods.

Features
--------

- Real-time construction of a 3D scene graph with open-set segmentation capabilities.
- Outputs the scene graph in a human-readable format.
- Designed with a user-friendly interface.
- Provides integration with ROS (Robot Operating System).