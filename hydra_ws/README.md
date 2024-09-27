# Hydra-ws

### Quick Start

- Docker build and exec into the container
    ```
    docker compose run --rm --build hydra-ws-build
    ```
- Run the container
    ```
    docker compose run --rm hydra-ws
    ```
- Build the scene graph by running the ros launch
    ```
    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch hydra_ros uhumans2.launch zmq_ip:=127.0.0.1
    ```