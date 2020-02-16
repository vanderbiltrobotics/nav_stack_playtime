#!/usr/bin/env python

import random
from math import *
import geometry_msgs.msg
import rospy
import tf_conversions
import tf2_ros


class Robot:

    def __init__(self, x, y, length, theta=0.0):
        self.length = float(length)
        self.x = x
        self.y = y
        self.orientation = theta
        self.position_error = 0.05
        self.orientation_error = 0.05
        self.dt = 0.1

    # simulate noise and uncertainty in the robot's motion. I'm using Gaussian noise for these
    def move(self, angle, dist):
        orientation = self.orientation + float(angle) + random.gauss(0.0, self.orientation_error) * self.dt
        orientation %= 2 * pi
        # distance to move
        # move, and add randomness to the motion command
        dist = (dist + random.gauss(0.0, self.position_error)) * self.dt
        self.x = self.x + (cos(orientation) * dist)
        self.y = self.y + (sin(orientation) * dist)
        self.orientation = orientation
        log = "Moved to position: (" + str(self.x) + ", " + str(self.y) + "), theta: " + str(self.orientation)
        rospy.loginfo(log)

    def set_noise(self, pos, theta):
        self.position_error = float(pos)
        self.orientation_error = float(theta)

    def get_pose(self):
        p = geometry_msgs.msg.PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "odom"
        p.pose.position.x = self.x
        p.pose.position.y = self.y
        p.pose.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.orientation)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p


def tf_from_pose(p):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation = p.pose.position
    t.transform.rotation = p.pose.orientation

    br.sendTransform(t)


def twist_command_handler(twist):
    theta = twist.angular.z
    vec = twist.linear.x
    my_robot.move(theta, vec)


if __name__ == '__main__':
    rospy.init_node('dummy_robot')
    pub = rospy.Publisher('odom1', geometry_msgs.msg.PoseStamped, queue_size=10)
    twist_sub = rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, twist_command_handler, queue_size=10)
    my_robot = Robot(0, 0, 0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        my_robot.set_noise(0, 0)
        # my_robot.move(0.001, 0.05)
        tf_from_pose(my_robot.get_pose())
        pub.publish(my_robot.get_pose())
        rate.sleep()

