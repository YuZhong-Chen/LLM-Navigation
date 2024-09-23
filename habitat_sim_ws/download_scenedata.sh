#!/bin/bash -e

SPLIT="val"

# Read command line arguments
while [[ $# -gt 0 ]]
do
key="$1"

    case $key in
        --split)
            SPLIT="$2"
            shift
            shift
            ;;
        --token-id)
            TOKEN_ID="$2"
            shift
            shift
            ;;
        --token-secret)
            TOKEN_SECRET="$2"
            shift
            shift
            ;;
        *)
            ;;
    esac
done

# Move to habitat sim workspace
cd /habitat-lab

# Echo 
echo "Downloading Scene Data for split: $SPLIT"
echo "Token ID: $TOKEN_ID"
echo "Token secret: $TOKEN_SECRET"

if [ ! -d "/home/user/ObjectSceneGraph/habitat_sim_ws/data/scenedata" ]; then

    echo "Downloading Scene Data for split: $SPLIT"

    # Check token
    if [ -z "$TOKEN_ID" ] || [ -z "$TOKEN_SECRET" ]; then
        echo "Please provide token id and token secret"
        exit 1
    fi

    # Activate habitat conda environment
    . activate habitat

    # Download Scene Data with habitat-sim python script
    python -m habitat_sim.utils.datasets_download --username $TOKEN_ID --password $TOKEN_SECRET --uids hm3d_$SPLIT\_v0.2
fi


# Check if data folder exists
echo "Linking Scene Data to habitat-lab"
if [ -d "/habitat-lab/data/versioned_data/hm3d-0.2/hm3d" ] && [ ! -d "/home/user/ObjectSceneGraph/habitat_sim_ws/data/scenedata" ]; then
    echo "Move scene data to mounted folder"
    mv /habitat-lab/data/versioned_data/hm3d-0.2/hm3d ~/ObjectSceneGraph/habitat_sim_ws/data/scenedata
fi

if [ ! -d "/habitat-lab/data/versioned_data/hm3d-0.2" ]; then
    mkdir -p /habitat-lab/data/versioned_data/hm3d-0.2
fi

if [ ! -d "/habitat-lab/data/scene_datasets" ]; then
    mkdir -p /habitat-lab/data/scene_datasets
fi

ln -f -s ~/ObjectSceneGraph/habitat_sim_ws/data/scenedata /habitat-lab/data/versioned_data/hm3d-0.2/hm3d
ln -f -s ~/ObjectSceneGraph/habitat_sim_ws/data/scenedata /habitat-lab/data/scene_datasets/hm3d