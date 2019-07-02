#!/usr/bin/env python

import sys
import rospy
import rospkg
import tf
import numpy as np
from math import cos, sin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

PKG_NAME = "motion_control"


# helper function to extract and place values into ROS message format
def quat_to_msg(quat, msg_vec):
    msg_vec.x = quat[0]
    msg_vec.y = quat[1]
    msg_vec.z = quat[2]
    msg_vec.w = quat[3]


class OdometryPublisher:
    def __init__(self, turning_radius, path=""):
        self.path = path
        self.odom_msg = Odometry()
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_footprint'
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.tfb = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()

        self.yaw = 0.0
        self.odom_frame_yaw = 0.0
        self.odom_pos = (0.0, 0.0)
        self.om_transform = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        self.weights = self._load_weights(turning_radius)
        self.acc_delta = np.zeros((1, 2))

    def save_weights(self):
        np.savetxt(self.path + "odom_weights", self.weights, fmt="%1.4f",
                   header="left_slippage, right_slippage, inverse_turning_radius",
                   footer="1.0 means no slippage, 0.75 means motion is reduced by 25% due to slippage")

    def _load_weights(self, turning_radius):
        try:
            weights = np.loadtxt(self.path + "odom_weights", ndmin=2)
            print("Odometry weights loaded")
        except:
            weights = np.array([1.0, 1.0, turning_radius])
            print("Default odometry weights - will learn from scratch")
            print(weights)
        return weights

    def publish_odom(self, left_speed, right_speed, dt, now):
        wheel_deltas = np.array([left_speed, right_speed]) * dt
        delta, theta = self._calc_greeks(wheel_deltas)
        self.acc_delta += wheel_deltas

        self.yaw += theta
        self.odom_msg.header.stamp = now
        delta_x = cos(self.yaw) * delta
        delta_y = sin(self.yaw) * delta
        self.odom_msg.pose.pose.position.x += delta_x
        self.odom_msg.pose.pose.position.y += delta_y
        dt = (1 if dt < 0.00001 else dt)
        self.odom_msg.twist.twist.angular.z = theta / dt
        self.odom_msg.twist.twist.linear.x = delta / dt
        # self.odom_msg.twist.twist.linear.y = delta_y / dt

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw) #, 'ryxz')
        quat_to_msg(quat, self.odom_msg.pose.pose.orientation)

        self.odom_pub.publish(self.odom_msg)
        # broadcast transform over tf
        self.odom_trans.header.stamp = now
        self.odom_trans.transform.translation = self.odom_msg.pose.pose.position
        self.odom_trans.transform.rotation = self.odom_msg.pose.pose.orientation
        self.tfb.sendTransformMessage(self.odom_trans)

    def _calc_greeks(self, wheel_deltas):
        slip_l = self.weights[0]
        slip_r = self.weights[1]
        delta = 0.5 * slip_l * wheel_deltas[0] + 0.5 * slip_r * wheel_deltas[1]
        theta = (-slip_l * wheel_deltas[0] + slip_r * wheel_deltas[1]) * self.weights[2]
        return delta[0], theta[0]
