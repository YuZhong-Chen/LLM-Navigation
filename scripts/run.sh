#!/bin/bash -e

function usage {
    echo "Usage: $0 [-n] [--only-odom]"
    echo "  -n: Run navigation node"
    echo "  --only-odom: Run only odometry node"
    exit 1
}

function run_odom {
    # Run kobuki driver node, set kobuki_ws_dir as $HOME/workspace/kobuki_ws/docker
    kobuki_ws_dir=$HOME/workspace/kobuki_ws/docker
    # Get into kobuki_ws directory and run kobuki driver node
    cd $kobuki_ws_dir
    docker compose run $1 --rm kobuki-ws /home/ros2-essentials/kobuki_ws/script/kobuki-bringup.sh
}

function run_lidar {
    # Run LiDAR driver node
    lidar_ws_dir=$HOME/workspace/vlplidar_ws/docker

    cd $lidar_ws_dir
    docker compose run $1 --rm vlplidar-ws /home/ros2-essentials/vlplidar_ws/scripts/lidar-bringup.sh
}

function run_realsense {
    # Run Realsense driver node
    realsense_ws_dir=$HOME/workspace/realsense_ros1_ws/docker

    cd $realsense_ws_dir
    docker compose run $1 --rm realsense-ros1-ws /home/ros2-essentials/realsense_ros1_ws/scripts/realsense-bringup.sh
}

function run_nav {
    # Run navigation node
    cd $kobuki_ws_dir
    docker compose run $1 --rm kobuki-ws /home/ros2-essentials/kobuki_ws/script/nav-bringup.sh
}

if [ "$1" == "-h" ]; then
    usage
    exit 0
fi

if [ "$1" == "--only-odom" ]; then
    run_odom
else 
    run_odom -d
    run_realsense -d
    if [ "$1" == "-n" ]; then
        run_lidar -d
    else
        run_lidar
    fi
fi

# Set first argument -n for setting if we need to run navigation
if [ "$1" == "-n" ]; then
    run_nav
fi

# Close containers
docker container stop $(docker container ls -q --filter name=docker-kobuki-ws)

# If not run only odom - remove lidar too
if [ "$1" != "--only-odom" ]; then
    docker container stop $(docker container ls -q --filter name=docker-vlplidar-ws)
fi

exit 0