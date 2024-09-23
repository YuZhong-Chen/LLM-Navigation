#!/bin/bash -e

# Move to habitat sim workspace
cd ~/ObjectSceneGraph/habitat_sim_ws

# Check if data/v1 folder not exists
if [ ! -d "data/v1" ]; then
    
    echo "Downloading HM3D object navigation dataset"
    
    # Download HM3D object navigation dataset
    wget https://dl.fbaipublicfiles.com/habitat/data/datasets/objectnav/hm3d/v1/objectnav_hm3d_v1.zip

    # Unzip the dataset and put it into correct folder
    unzip objectnav_hm3d_v1.zip -d data
    mv data/objectnav_hm3d_v1 data/v1
    rm objectnav_hm3d_v1.zip
fi

echo "Linking HM3D object navigation dataset to habitat-lab"

# Check if data folder not exists
if [ ! -d "/habitat-lab/data/datasets/objectnav/hm3d" ]; then
    
    # Create the folder
    mkdir -p /habitat-lab/data/datasets/objectnav/hm3d
fi
    
# Link the dataset to habitat sim
ln -s ~/ObjectSceneGraph/habitat_sim_ws/data/v1 /habitat-lab/data/datasets/objectnav/hm3d/v1