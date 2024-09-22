ROS2 Humble
===========

.. note::

    For more information about installation, please visit the official `ROS2 website <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`_.

Please note that we have already installed ROS 2 Humble within the Docker environment. 
Generally, you do not need to install ROS 2 yourself. You will only need to manually install ROS 2 in a few exceptional cases, 
such as when you want to add features to our codebase but have your own Dockerfile.

Follow these steps to install ROS 2 Humble on your system:

1. **Set Up Your System**
    Ensure that your system is up to date::

        sudo apt update
        sudo apt upgrade
        sudo apt install -y software-properties-common
        sudo add-apt-repository universe

2. **Add the ROS2 GPG Key**  
    Download and add the GPG key for the ROS 2 repository::

        sudo apt install -y curl
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

3. **Add the ROS 2 Repository**  
    Add the ROS 2 repository to your sources list::

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


4. **Install ROS 2 Humble**  
    Update your package list and install the full desktop version of ROS 2 Humble::

        sudo apt update
        sudo apt upgrade
        sudo apt install ros-humble-desktop


5. **Set Up Environment Variables**  
    Add the ROS 2 environment variables to your shell::

        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        source ~/.bashrc
