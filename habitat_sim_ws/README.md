# Habitat-sim

### Quick Start

- Docker build and exec into the container
    ```
    docker compose up -d --build
    docker exec -it habitat-sim-ws bash
    ```
- Run install script for outer packages
    ```
    ./install.sh
    ```
- Download object navigation dataset
    ```
    ./download_objectnav.sh
    ```
- Download HM3D scene dataset (Please request the dataset first and create API key)
    ```
    ./download_scenedata.sh --token-id <TOKEN_ID> --token-secret <TOKEN_SECRET>
    ```
- Change the dataset from train to val
    ```
    sudo vim /habitat-lab/habitat-lab/habitat/config/habitat/dataset/objectnav/hm3d.yaml
    ```
    - Modify `train` in line 7 into `val`
        ```
        # @package habitat.dataset
        defaults:
        - /habitat/dataset: dataset_config_schema
        - _self_

        type: ObjectNav-v1
        split: val
        data_path: data/datasets/objectnav/hm3d/v1/{split}/{split}.json.gz
        ```
- Run roscore in background
    ```
    roscore &
    ```
- Run the node
    ```
    ./run.sh
    ```