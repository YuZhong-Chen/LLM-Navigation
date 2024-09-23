#!/usr/bin/env python3

# ROS
import habitat.utils
import habitat.utils.geometry_utils
import rospy

# ROS Messages
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
import geometry_msgs.msg

# TF
import tf.transformations as tf

# Habitat
import gym
import habitat
import habitat.gym
from habitat.sims.habitat_simulator.actions import HabitatSimActions
from habitat.core.logging import logger
import quaternion

# OpenCV
import cv2
import cv_bridge

# Other utilities
import time
import numpy as np
import threading
from typing import Dict
from collections import defaultdict

FORWARD_KEY = "w"
LEFT_KEY = "a"
RIGHT_KEY = "d"
FINISH = "q"
LOOK_UP = "e"
LOOK_DOWN = "c"


class HabitatSimNode(object):
    def __init__(self, publish_rate: float = 10.0) -> None:
        # Initialize the node
        rospy.init_node("habitat_sim_test", anonymous=True)

        # Subscribers
        self.action_sub = rospy.Subscriber("/main_program/action", String, self.ActionCallback)
        self.request_sub = rospy.Subscriber("/habitat_sim/request_command", String, self.RequestCallback)

        # Publishers
        self.rgb_image_pub = rospy.Publisher("/habitat_sim/rgb/image", Image, queue_size=10)
        self.rgb_info_pub = rospy.Publisher("/habitat_sim/rgb/camera_info", CameraInfo, queue_size=10)
        self.depth_image_pub = rospy.Publisher("/habitat_sim/depth/image", Image, queue_size=10)
        self.depth_info_pub = rospy.Publisher("/habitat_sim/depth/camera_info", CameraInfo, queue_size=10)
        self.odom_pub = rospy.Publisher("/habitat_sim/odom", Odometry, queue_size=10)
        self.collision_pub = rospy.Publisher("/habitat_sim/collision", Bool, queue_size=10)
        self.object_name_pub = rospy.Publisher("/habitat_sim/objectgoal/name", String, queue_size=10)
        self.object_label_index_pub = rospy.Publisher("/habitat_sim/objectgoal/index", Int32, queue_size=10)
        self.sematic_image_pub = rospy.Publisher("/habitat_sim/semantic/image", Image, queue_size=10)
        self.sematic_object_name_pub = rospy.Publisher("/habitat_sim/semantic/object_name", String, queue_size=10)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / publish_rate), self.TimerCallback)

        # Thread Lock
        self.lock = threading.Lock()

        # Frame ID
        self.camera_frame_id = "habitat_sim_camera_frame"
        self.agent_frame_id = "odom"

        # OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Variables
        self.rgb_image = None
        self.rgb_camera_info = None
        self.depth_image = None
        self.depth_camera_info = None
        self.depth_scale = 10.0  # Transform the depth image to meters
        self.semantic_image = None
        self.odom = None
        self.action_command = "empty"
        self.previous_odom = None
        self.collision_msg = None
        self.objectname = None
        self.up_times = 0  # Each time -> 30 degrees
        self.quaternion = geometry_msgs.msg.Quaternion()
        self.semantic_object_name = {}
        self.object_label_number = None

        # Fill the quaternion for the orientation
        self.quaternion.x = 0.0
        self.quaternion.y = 0.0
        self.quaternion.z = 0.0
        self.quaternion.w = 1.0

    def ActionCallback(self, msg: String) -> None:
        self.action_command = msg.data

        # Print the action
        rospy.loginfo(f"Received action: {self.action_command}")

    def RequestCallback(self, msg: String) -> None:
        if msg.data == "PublishSemanticObjectName":
            self.PublishSemanticObjectName()
        else:
            rospy.logwarn(f"Invalid request command: {msg.data}")

    def TimerCallback(self, event) -> None:
        with self.lock:
            self.PublishData()

    def UpdateQuaternion(self) -> None:

        # Get the quaternion from the euler angles
        euler = tf.euler_from_quaternion([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        # Update the quaternion
        self.quaternion = tf.quaternion_from_euler(roll, pitch + self.up_times * np.pi / 6, yaw)

        # Convert to geometry_msgs.msg.Quaternion
        self.quaternion = geometry_msgs.msg.Quaternion(x=self.quaternion[0], y=self.quaternion[1], z=self.quaternion[2], w=self.quaternion[3])

    def UpdateSemanticObjectName(self, object_name: Dict) -> None:
        self.semantic_object_name = object_name

    def UpdateRGBImage(self, rgb_image: np.ndarray) -> None:
        with self.lock:
            self.rgb_image = rgb_image

    def UpdateRGBCameraInfo(self, rgb_camera_info: CameraInfo) -> None:
        with self.lock:
            self.rgb_camera_info = rgb_camera_info

    def UpdateDepthImage(self, depth_image: np.ndarray) -> None:
        with self.lock:
            self.depth_image = depth_image * self.depth_scale

    def UpdateDepthCameraInfo(self, depth_camera_info: CameraInfo) -> None:
        with self.lock:
            self.depth_camera_info = depth_camera_info

    def UpdateSemanticImage(self, semantic_image: np.ndarray) -> None:
        with self.lock:
            self.semantic_image = semantic_image

    def UpdateOdometry(self, odom: Odometry) -> None:
        with self.lock:
            self.odom = odom

    def UpdateCollision(self, collision: bool) -> None:
        with self.lock:
            self.collision_msg = collision

    def UpdateObjectName(self, objectname: String) -> None:
        with self.lock:
            self.objectname = objectname

    def UpdateObjectLabelNumber(self, object_label_number: Int32) -> None:
        with self.lock:
            self.object_label_number = object_label_number

    def CheckCollision(self) -> bool:
        if self.odom is None:
            return False

        if self.previous_odom is None:
            self.previous_odom = self.odom
            return False

        previous_position = self.previous_odom.pose.pose.position
        current_position = self.odom.pose.pose.position

        collision = False
        if previous_position.x == current_position.x and previous_position.y == current_position.y and previous_position.z == current_position.z:
            collision = True

        self.previous_odom = self.odom

        return collision

    def decode_name(self, name) -> tuple[str, int]:
        return name.split("_")[0], int(name.split("_")[1])

    def PublishData(self) -> None:
        timestamp = rospy.Time.now()

        # Publish RGB Image
        if self.rgb_image is not None:
            rgb_image_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="bgr8")
            rgb_image_msg.header.stamp = timestamp
            rgb_image_msg.header.frame_id = self.camera_frame_id
            self.rgb_image_pub.publish(rgb_image_msg)

        # Publish RGB Camera Info
        if self.rgb_camera_info is not None:
            self.rgb_camera_info.header.stamp = timestamp
            self.rgb_camera_info.header.frame_id = self.camera_frame_id
            self.rgb_info_pub.publish(self.rgb_camera_info)

        # Publish Depth Image
        if self.depth_image is not None:
            depth_image_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="32FC1")
            depth_image_msg.header.stamp = timestamp
            depth_image_msg.header.frame_id = self.camera_frame_id
            self.depth_image_pub.publish(depth_image_msg)

        # Publish Depth Camera Info
        if self.depth_camera_info is not None:
            self.depth_camera_info.header.stamp = timestamp
            self.depth_camera_info.header.frame_id = self.camera_frame_id
            self.depth_info_pub.publish(self.depth_camera_info)

        # Publish Semantic Image
        if self.semantic_image is not None:

            # Convert from 16 bit to 8 bit
            self.semantic_image = self.semantic_image.astype(np.uint8)

            semantic_image_msg = self.bridge.cv2_to_imgmsg(self.semantic_image, encoding="rgb8")
            semantic_image_msg.header.stamp = timestamp
            semantic_image_msg.header.frame_id = self.camera_frame_id
            self.sematic_image_pub.publish(semantic_image_msg)

        # Publish Odometry
        if self.odom is not None:
            self.odom.header.stamp = timestamp
            self.odom.header.frame_id = self.agent_frame_id
            self.odom_pub.publish(self.odom)

        # Publish Collision
        if self.collision_msg is not None:
            self.collision_pub.publish(self.collision_msg)

        # Publish Object name
        if self.objectname is not None:
            self.object_name_pub.publish(self.objectname)

        # Publish Object label number
        if self.object_label_number is not None:
            self.object_label_index_pub.publish(self.object_label_number)

    def PublishSemanticObjectName(self) -> None:
        if len(self.semantic_object_name) == 0:
            rospy.logwarn("Semantic object name is empty.")
            return

        # Construct the semantic object name
        # Since ROS does not support String Array, we will use a delimiter to separate the names
        string_data = ""
        for key, value in self.semantic_object_name.items():
            string_data += str(key) + ":" + value + ","
        string_data = string_data[:-1]

        # Publish the semantic object name
        semantic_object_name_msg = String()
        semantic_object_name_msg.data = string_data
        self.sematic_object_name_pub.publish(semantic_object_name_msg)

def deal_mapping(mapping):
    semantic_map = {}
    conversion_map = {}

    current_key = 0
    for key, value in mapping.items():
        if value not in semantic_map.values():
            semantic_map[current_key] = value
            current_key += 1
        
        # Convert the key to the new key
        conversion_map[key] = list(semantic_map.keys())[list(semantic_map.values()).index(value)]

    # Log the semantic map
    for key, value in semantic_map.items():
        print(key, "->", value)

    return semantic_map, conversion_map

def write_csv(path, mapping):

    # Sth like
    # name,red,green,blue,alpha,id
    # floor,0,0,0,255,0
    # chair,128,0,0,255,1
    idx = 0
    with open(path, "w") as file:
        file.write("name,red,green,blue,alpha,id\n")
        for key, value in mapping.items():
            file.write(f"{value[1:-1]},{key},{key},{key},255,{idx}\n")
            idx += 1

def write_label_name(path, mapping):

    # Sth like
    # - {label: 0, name: floor}
    # - {label: 1, name: chair}
    idx = 0
    with open(path, "w") as file:
        for key, value in mapping.items():
            file.write(f"- {{label: {idx}, name: {value[1:-1]}}}\n")
            idx += 1

def main():
    # Setup the seed
    # seed = np.random.randint(1, 100)
    seed = 197

    # Setup the config path
    # Habitat default config path: /habitat-lab/habitat-lab/habitat/config/benchmark/nav/objectnav/objectnav_hm3d.yaml
    # habitat_config = habitat.get_config(config_path="benchmark/nav/objectnav/objectnav_hm3d.yaml", overrides=["habitat.seed=" + str(seed)])
    habitat_config = habitat.get_config(
        config_path="/home/user/ObjectSceneGraph/habitat_sim_ws/config/objectnav_hm3d.yaml",
        configs_dir="/",
        overrides=[
            "habitat.seed=" + str(seed),
            "habitat.task.measurements.success.success_distance=0.18",
        ],
    )

    # Create the environment and initialize
    env = habitat.Env(config=habitat_config)
    observations = env.reset()

    # # Print the environment configuration
    # env_config = env._config
    # for key in env_config.keys():
    #     print(key, ":", env_config[key])
    # 0 -> armchair_71/chair
    # 1 -> bed_17
    # 3 -> toilet
    # 4 -> tv_403/monitor
    # 5 -> couch_34

    # Create the node
    habitat_sim_node = HabitatSimNode()

    # Get the camera position and orientation, this is used to publish the base_link to camera_link transform
    # Please based on these values modify the transform in the habitat_fake_odom config file.
    camera_position = env._config["simulator"]["agents"]["main_agent"]["sim_sensors"]["rgb_sensor"]["position"]
    camera_orientation = env._config["simulator"]["agents"]["main_agent"]["sim_sensors"]["rgb_sensor"]["orientation"]
    agent_height = env._config["simulator"]["agents"]["main_agent"]["height"]
    print("Remember to modify the transform in the habitat_fake_odom config file. (base_link -> camera_link)")
    print("Camera Position:", camera_position)
    print("Camera Orientation:", camera_orientation)
    print("Agent Height:", agent_height)

    # RGB Image
    observations["rgb"] = cv2.cvtColor(observations["rgb"], cv2.COLOR_RGB2BGR)
    habitat_sim_node.UpdateRGBImage(observations["rgb"])
    cv2.imshow("RGB", observations["rgb"])

    # Depth Image
    # NOTE: Check the units of the depth image, and modify the depth scale in the HabitatSimNode accordingly
    habitat_sim_node.UpdateDepthImage(observations["depth"])

    # Get RGB Camera Matrix
    # Reference:
    # - https://aihabitat.org/docs/habitat-lab/view-transform-warp.html#intrinsic-parameters-k
    rgb_camera_config = env._config["simulator"]["agents"]["main_agent"]["sim_sensors"]["rgb_sensor"]
    rgb_camera_height = rgb_camera_config["height"]
    rgb_camera_width = rgb_camera_config["width"]
    rgb_camera_fov = rgb_camera_config["hfov"]
    rgb_camera_fx = rgb_camera_fy = rgb_camera_width / (2 * np.tan(np.deg2rad(rgb_camera_fov) / 2))
    rgb_camera_cx = rgb_camera_width / 2
    rgb_camera_cy = rgb_camera_height / 2

    # Update RGB Camera Info
    rgb_camera_info = CameraInfo()
    rgb_camera_info.height = rgb_camera_height
    rgb_camera_info.width = rgb_camera_width
    rgb_camera_info.distortion_model = "plumb_bob"
    rgb_camera_info.K = [rgb_camera_fx, 0.0, rgb_camera_cx, 0.0, rgb_camera_fy, rgb_camera_cy, 0.0, 0.0, 1.0]
    rgb_camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    rgb_camera_info.P = [rgb_camera_fx, 0.0, rgb_camera_cx, 0.0, 0.0, rgb_camera_fy, rgb_camera_cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    habitat_sim_node.UpdateRGBCameraInfo(rgb_camera_info)

    # Get Depth Camera Matrix
    depth_camera_config = env._config["simulator"]["agents"]["main_agent"]["sim_sensors"]["depth_sensor"]
    depth_camera_height = depth_camera_config["height"]
    depth_camera_width = depth_camera_config["width"]
    depth_camera_fov = depth_camera_config["hfov"]
    depth_camera_fx = depth_camera_fy = depth_camera_width / (2 * np.tan(np.deg2rad(depth_camera_fov) / 2))
    depth_camera_cx = depth_camera_width / 2
    depth_camera_cy = depth_camera_height / 2

    # Update Depth Camera Info
    depth_camera_info = CameraInfo()
    depth_camera_info.height = depth_camera_height
    depth_camera_info.width = depth_camera_width
    depth_camera_info.distortion_model = "plumb_bob"
    depth_camera_info.K = [depth_camera_fx, 0.0, depth_camera_cx, 0.0, depth_camera_fy, depth_camera_cy, 0.0, 0.0, 1.0]
    depth_camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    depth_camera_info.P = [depth_camera_fx, 0.0, depth_camera_cx, 0.0, 0.0, depth_camera_fy, depth_camera_cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    habitat_sim_node.UpdateDepthCameraInfo(depth_camera_info)

    # Transform agent position to ROS odometry
    # NOTE: The habitat-sim uses a different coordinate system than ROS
    gps_data = observations["gps"]
    quaternion_data = quaternion.from_euler_angles(0, 0, observations["compass"][0])
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = gps_data[0]
    odom_msg.pose.pose.position.y = -gps_data[1]
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation = quaternion_data
    habitat_sim_node.UpdateOdometry(odom_msg)

    # Update object name
    objectname = String()
    objectlabel = Int32()
    objectname.data, objectlabel.data = habitat_sim_node.decode_name(env._current_episode.goals[0].object_name)
    habitat_sim_node.UpdateObjectName(objectname)
    habitat_sim_node.UpdateObjectLabelNumber(objectlabel)

    # Get scene information from habitat-sim
    scene_path = env.current_episode.scene_id.split("/")[:-1]
    scene_path = "/".join(scene_path)
    scene_path = "/habitat-lab/" + scene_path
    scene_name = env.current_episode.scene_id.split("/")[-1]
    scene_name = scene_name.split(".")[0]
    scene_name = scene_name + ".semantic.txt"
    scene_path = scene_path + "/" + scene_name
    print("Scene Path:", scene_path)

    # Get mapping for the scene txt
    semantic_map = {}
    with open(scene_path, "r") as file:
        for line in file:
            line = line.strip()
            line = line.split(",")
            if len(line) != 4:
                continue
            semantic_map[int(line[0])] = line[2]
    habitat_sim_node.UpdateSemanticObjectName(semantic_map)
    semantic_map, conversion_map = deal_mapping(semantic_map)
    write_csv("/home/user/ObjectSceneGraph/habitat_sim_ws/config/semantic_map.csv", semantic_map)
    write_label_name("/home/user/ObjectSceneGraph/habitat_sim_ws/config/semantic_label_name.yaml", semantic_map)

    # Print the semantic map
    print("Semantic Map:")
    for key, value in semantic_map.items():
        print(key, "->", value)

    # Create the semantic color mapping table
    semantic_color_table = {}
    for key, _ in semantic_map.items():
        if key not in semantic_color_table:
            # Pick a random color which is not in the table
            random_color = np.random.randint(1, 255, size=3).tolist()
            while random_color in semantic_color_table.values():
                random_color = np.random.randint(1, 255, size=3).tolist()

            semantic_color_table[key] = random_color

    # Update the semantic map
    semantic_map = observations["semantic"].astype(np.uint16)

    # Convert the semantic map to the new key
    # for key, value in conversion_map.items():
    #     semantic_map[(semantic_map == key).squeeze()] = value
    # Optimized version
    semantic_map = np.vectorize(lambda x: conversion_map.get(x, 0))(semantic_map)

    # Convert 1D to 3D
    # self.semantic_image = np.repeat(self.semantic_image[:, :, np.newaxis], 3, axis=2)
    semantic_map = np.repeat(semantic_map[:, :, np.newaxis], 3, axis=2)

    habitat_sim_node.UpdateSemanticImage(semantic_map)
    # semantic_map_rgb = np.zeros((semantic_map.shape[0], semantic_map.shape[1], 3), dtype=np.uint8)
    # for key, value in semantic_color_table.items():
    #     semantic_map_rgb[(semantic_map == key).squeeze()] = value
    # cv2.imshow("Semantic", semantic_map_rgb)

    # Print the object goal
    print(f"Target Object: {observations['objectgoal']} {env._current_episode.goals[0].object_name}")

    # Main loop
    count_steps = 0
    while not env.episode_over:
        action = None

        # Read from the keyboard
        keystroke = cv2.waitKey(1)
        if keystroke == ord(FORWARD_KEY):
            action = HabitatSimActions.move_forward
            print("action: FORWARD")
        elif keystroke == ord(LEFT_KEY):
            action = HabitatSimActions.turn_left
            print("action: LEFT")
        elif keystroke == ord(RIGHT_KEY):
            action = HabitatSimActions.turn_right
            print("action: RIGHT")
        elif keystroke == ord(LOOK_UP):
            action = HabitatSimActions.look_up
            habitat_sim_node.up_times += 1
            print("action: LOOK UP")
        elif keystroke == ord(LOOK_DOWN):
            action = HabitatSimActions.look_down
            habitat_sim_node.up_times -= 1
            print("action: LOOK DOWN")
        elif keystroke == ord(FINISH) or keystroke == 27:
            action = HabitatSimActions.stop
            print("action: FINISH")
        elif keystroke != -1:
            print("INVALID KEY")
            continue

        # Read from the action subscriber
        if habitat_sim_node.action_command != "empty":
            if habitat_sim_node.action_command == "forward":
                action = HabitatSimActions.move_forward
            elif habitat_sim_node.action_command == "left":
                action = HabitatSimActions.turn_left
            elif habitat_sim_node.action_command == "right":
                action = HabitatSimActions.turn_right
            elif habitat_sim_node.action_command == "look_up":
                action = HabitatSimActions.look_up
                habitat_sim_node.up_times += 1
            elif habitat_sim_node.action_command == "look_down":
                action = HabitatSimActions.look_down
                habitat_sim_node.up_times -= 1
            elif habitat_sim_node.action_command == "finish":
                action = HabitatSimActions.stop

            habitat_sim_node.action_command = "empty"

        if action is None:
            continue

        observations = env.step(action)
        count_steps += 1
        print("Step:", count_steps)

        # Log the image size
        print("RGB Image Size:", observations["rgb"].shape)

        # RGB Image
        observations["rgb"] = cv2.cvtColor(observations["rgb"], cv2.COLOR_RGB2BGR)
        cv2.imshow("RGB", observations["rgb"])
        habitat_sim_node.UpdateRGBImage(observations["rgb"])

        # Depth Image
        habitat_sim_node.UpdateDepthImage(observations["depth"])

        # Semantic Image
        semantic_map = observations["semantic"].astype(np.uint16)

        # Convert the semantic map to the new key
        # for key, value in conversion_map.items():
        #     semantic_map[(semantic_map == key).squeeze()] = value
        # Optimized version
        semantic_map = np.vectorize(lambda x: conversion_map.get(x, 0))(semantic_map)

        # Convert 1D to 3D
        # self.semantic_image = np.repeat(self.semantic_image[:, :, np.newaxis], 3, axis=2)
        semantic_map = np.repeat(semantic_map[:, :, np.newaxis], 3, axis=2)
        # Squeeze the semantic map
        semantic_map = np.squeeze(semantic_map)

        print("Semantic Image Size:", semantic_map.shape)

        habitat_sim_node.UpdateSemanticImage(semantic_map)
        # semantic_map_rgb = np.zeros((semantic_map.shape[0], semantic_map.shape[1], 3), dtype=np.uint8)
        # for key, value in semantic_color_table.items():
        #     semantic_map_rgb[(semantic_map == key).squeeze()] = value
        # cv2.imshow("Semantic", semantic_map_rgb)

        # Transform agent position to ROS odometry
        # NOTE: The habitat-sim uses a different coordinate system than ROS
        gps_data = observations["gps"]
        quaternion_data = quaternion.from_euler_angles(0, 0, observations["compass"][0])
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = gps_data[0]
        odom_msg.pose.pose.position.y = -gps_data[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_data
        habitat_sim_node.UpdateOdometry(odom_msg)

        # Check collision only when forward action is taken
        collision_msg = Bool()
        if action == HabitatSimActions.move_forward:
            collision_msg.data = habitat_sim_node.CheckCollision()
        else:
            collision_msg.data = False
        habitat_sim_node.UpdateCollision(collision_msg)

    logger.info("Episode seed: {}".format(seed))
    logger.info("Total steps: {}".format(count_steps))
    agg_metrics: Dict = defaultdict(float)
    metrics = env.get_metrics()
    for m, v in metrics.items():
        if isinstance(v, dict):
            for sub_m, sub_v in v.items():
                agg_metrics[m + "/" + str(sub_m)] += sub_v
        else:
            agg_metrics[m] += v

    for k, v in metrics.items():
        logger.info("{}: {}".format(k, round(v, 3)))


if __name__ == "__main__":
    main()
