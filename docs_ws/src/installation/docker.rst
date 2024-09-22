Docker
======

.. note::

    For more information about installation, please visit the official `docker website <https://docs.docker.com/engine/install/ubuntu/>`_.

To ensure better maintainability of our code, we have packaged all the necessary packages and dependencies into Docker and split it into multiple workspaces based on functionality. 
Normally, you only need to navigate to the designated workspace and use `docker compose up` to complete the entire environment setup.

Follow these steps to install Docker and Docker Compose on your system:


1. **Update your package index**::

    sudo apt-get update

2. **Install required packages**::
    
    sudo apt install apt-transport-https ca-certificates curl software-properties-common

3. **Add the Docker GPG key**::
    
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

4. **Add the Docker repository**::

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

5. **Update your package index again**::

    sudo apt-get update

6. **Install Docker**::

    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

7. **Verify the installation**::

    sudo docker --version