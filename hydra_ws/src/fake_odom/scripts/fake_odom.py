#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

# TF
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Clock
from rosgraph_msgs.msg import Clock

current_time = None

def odom_callback(msg):

    # if current_time is None:
    #     return

    # br = tf.TransformBroadcaster()
    br = tf2_ros.StaticTransformBroadcaster()

    # Create TF message
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "odom"
    # t.child_frame_id = "base_link"
    t.header.frame_id = "world"
    t.child_frame_id = "base_link_gt"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    # print("Current time: sec: {}, nsec: {}".format(current_time.secs, current_time.nsecs))

    # Broadcast the TF
    br.sendTransform(t)
    
    # Send static TF map -> odom
    t.header.frame_id = "map"
    # t.child_frame_id = "odom"
    t.child_frame_id = "world"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    br.sendTransform(t)

    # Broadcast the TF: base_link_gt -> left_cam
    # t.header.frame_id = "base_link"
    # t.child_frame_id = "camera_color_optical_frame"
    t.header.frame_id = "base_link_gt"
    t.child_frame_id = "left_cam"
    t.transform.translation.x = 0
    t.transform.translation.y = 0.05
    # t.transform.translation.z = 0.5
    t.transform.translation.z = 0.8
    t.transform.rotation.x = 0.5
    t.transform.rotation.y = -0.5
    t.transform.rotation.z = 0.5
    t.transform.rotation.w = -0.5

    br.sendTransform(t)

    # Broadcast the TF: base_link_gt -> right_cam
    # t.header.frame_id = "base_link"
    # t.child_frame_id = "camera_depth_optical_frame"
    t.header.frame_id = "base_link_gt"
    t.child_frame_id = "right_cam"
    t.transform.translation.x = 0
    # t.transform.translation.y = 0.05
    # t.transform.translation.z = 0.5
    t.transform.translation.y = -0.05
    t.transform.translation.z = 0.8
    t.transform.rotation.x = 0.5
    t.transform.rotation.y = -0.5
    t.transform.rotation.z = 0.5
    t.transform.rotation.w = -0.5

    br.sendTransform(t)


def clock_callback(msg):

    # Update the current time
    global current_time
    current_time = msg.clock


def fake_odom():

    rospy.init_node('fake_odom', anonymous=True)
    rospy.loginfo("[Fake Odom]: Fake odom node started")

    # Initialize the subscriber
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/clock", Clock, clock_callback)

    # Initialize the broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.spin()

if __name__ == '__main__':

    try:
        fake_odom()
    except rospy.ROSInterruptException:
        pass
