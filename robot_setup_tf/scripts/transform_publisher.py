#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from geometry_msgs.msg import Pose

from tf import transformations as tft
from tf import TransformerROS

# Import other required packages
import json
import sys

import numpy as np

# Paths to transform file directories
PATH_TO_DIR_STATIC = sys.path[0] + "/tf_configs_static/"
PATH_TO_DIR_DYNAMIC = sys.path[0] + "/tf_configs_dynamic/"


# Converts a list of transforms stored in dictionaries to a list of transform messages
def get_transform_msgs(transforms):

    # List to store messages
    transform_msgs = []

    # Create a tf message from each tf in input list
    for transform in transforms:

        # Create new tf message
        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()

        # Extract parent and child frame ids
        tf_msg.header.frame_id = transform["frame_id"]
        tf_msg.child_frame_id = transform["child_id"]

        # Extract translation
        tf_msg.transform.translation.x = transform["trans_x"]
        tf_msg.transform.translation.y = transform["trans_y"]
        tf_msg.transform.translation.z = transform["trans_z"]

        # Extract rotation
        tf_msg.transform.rotation.x = transform["rot_x"]
        tf_msg.transform.rotation.y = transform["rot_y"]
        tf_msg.transform.rotation.z = transform["rot_z"]
        tf_msg.transform.rotation.w = transform["rot_w"]

        # Add to the list
        transform_msgs.append(tf_msg)

    # Return the list of tf msgs
    return transform_msgs


class PoseToTF():

    # Constructor
    def __init__(self, pose_topic, child_id, frame_id, invert):

        # Subscriber to pose topic
        self.pose_sub = rospy.Subscriber(pose_topic, Pose, self.tf_from_pose)

        # TF broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Create base transform message
        self.tf_msg = tf2_ros.TransformStamped()
        self.tf_msg.header.frame_id = frame_id
        self.tf_msg.child_frame_id = child_id

        # Does this transform need to be inverted
        self.invert = invert

    # Callback function for pose messages. Converts to tf msg and publishes
    def tf_from_pose(self, pose_msg):

        # Update message timestamp
        self.tf_msg.header.stamp = rospy.Time.now()

        # Update translation
        self.tf_msg.transform.translation.x = pose_msg.position.x
        self.tf_msg.transform.translation.y = pose_msg.position.y
        self.tf_msg.transform.translation.z = pose_msg.position.z

        # Update rotation
        self.tf_msg.transform.rotation.x = pose_msg.orientation.x
        self.tf_msg.transform.rotation.y = pose_msg.orientation.y
        self.tf_msg.transform.rotation.z = pose_msg.orientation.z
        self.tf_msg.transform.rotation.w = pose_msg.orientation.w


        if sum ([self.tf_msg.transform.rotation.x, self.tf_msg.transform.rotation.y,
                self.tf_msg.transform.rotation.z, self.tf_msg.transform.rotation.w]) != 0.0:

            if self.invert == "True":

                # Create transformerROS and add our transform
                transformer = TransformerROS()
                transformer.setTransform(self.tf_msg)

                # Lookup the transform
                (trans, rot) = transformer.lookupTransform(self.tf_msg.header.frame_id, self.tf_msg.child_frame_id, rospy.Time(0))

                # Create transform object
                transform = tft.concatenate_matrices(tft.translation_matrix(trans), tft.quaternion_matrix(rot))

                # Invert
                inverse_tf = tft.inverse_matrix(transform)

                # Get translation, rotation vectors back out
                translation = tft.translation_from_matrix(inverse_tf)
                quaternion = tft.quaternion_from_matrix(inverse_tf)

                # Get RPY
                rpy = tft.euler_from_quaternion(quaternion)
                rpy_new = [rpy[0], rpy[1], -rpy[2]]

                # Back to quaternion
                quaternion = tft.quaternion_from_euler(rpy_new[0], rpy_new[1], rpy_new[2])

                # Update translation
                self.tf_msg.transform.translation.x = translation[0]
                self.tf_msg.transform.translation.y = -translation[1]
                self.tf_msg.transform.translation.z = translation[2]

                # Update rotation
                self.tf_msg.transform.rotation.x = quaternion[0]
                self.tf_msg.transform.rotation.y = quaternion[1]
                self.tf_msg.transform.rotation.z = quaternion[2]
                self.tf_msg.transform.rotation.w = quaternion[3]


            # Publish transform
            self.broadcaster.sendTransform(self.tf_msg)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transform_publisher")

    # Read in the name of transform file from parameter server
    file_name_static = rospy.get_param("tf_file_static", default="full_system.json")
    file_name_dynamic = rospy.get_param("tf_file_dynamic", default="full_system.json")

    # Load static transform file contents into a dictionary
    with open(PATH_TO_DIR_STATIC + file_name_static) as f:
        transform_dict_static = json.load(f)

    # Load dynamic transform file contents into a dictionary
    with open(PATH_TO_DIR_DYNAMIC + file_name_dynamic) as f:
        transform_dict_dynamic = json.load(f)

    # Extract lists of transform objects
    transform_list_static = transform_dict_static["transforms"]
    transform_list_dynamic = transform_dict_dynamic["transforms"]

    # Create a broadcaster for publishing static transforms
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Convert the list to transform messages, broadcast them
    broadcaster.sendTransform(get_transform_msgs(transform_list_static))

    # Create a PoseToTF object for each pose topic specified in file
    for transform in transform_list_dynamic:

        # Extract data from dictionary
        frame_id = transform["frame_id"]
        child_id = transform["child_id"]
        pose_topic = transform["pose_topic"]
        invert = transform["invert"]

        # Create a new PoseToTF object
        PoseToTF(pose_topic, child_id, frame_id, invert)

    # Spin indefinitely
    rospy.spin()

