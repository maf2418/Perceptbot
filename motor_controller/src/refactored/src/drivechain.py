#!/usr/bin/env python

import sys
import rospy
import rospkg
import tf
import numpy as np

from wheelencoder2 import Encoder
from odometry2 import OdometryPublisher, TwoWheelOdometryCalculator
from pidcontroller2 import PIDController
from geometry_msgs.msg import Twist
from motion_control.msg import PID_out, PID_settings
from utilities import *

PKG_NAME = "motion_control"

class DriveChainLayout:

    def setup_PIDs(self):
        pass

    def set_active_controller(self, controller=5):
        self.active_controller = controller

    @staticmethod 
    def load_encoder_settings(no_of_wheels):
        try:
            with open(ENCODER_SETTINGS_FILE) as f:
                encoder_pin_list = [int(x) for x in next(f).split()]
            print(encoder_pin_list)
            if len(encoder_pin_list) != no_of_wheels:
                raise RuntimeError
            encoder_list = [Encoder(gpio_pin) for gpio_pin in encoder_pin_list]
            return encoder_list
        except:
            RuntimeError


class TwoWheelDriveChain(DriveChainLayout):
    ''' 
    This is the only (sub)class which knows about
    the layout of the Perceptbot. The DriveChain knows
    which PIDController corresponds to which wheel. 
    In this implementation we have two wheels.
    
    This abstraction means that all other classes can be used
    independently of robot architecture.
    '''

    def __init__(self, left_speed=0.0, right_speed=0.0, integral_left=0.0, \
            integral_right=0.0, turning_radius=DEFAULT_TURNING_RADIUS):
        self.pub_l = rospy.Publisher("PID_l", PID_out, queue_size=10)
        self.pub_r = rospy.Publisher("PID_r", PID_out, queue_size=10)
        self.data_msg_l = PID_out(wheel="left")
        self.data_msg_r = PID_out(wheel="right")
        self.no_of_wheels = 2   # so we can load correct params
        self.prev_pwms = (0, 0)

        self.l_controller = PIDController(speed=left_speed, integral_speed=integral_left, name="left")
        self.r_controller = PIDController(speed=right_speed, integral_speed=integral_right, name="right")

        # set up controllers with encoders
        self.load_encoder_settings()

        # set up odometry
        self.odom_publisher = OdometryPublisher(turning_radius, TwoWheelOdometryCalculator)

    def load_encoder_settings(self):
        encoder_list = DriveChainLayout.load_encoder_settings(2)
        l_encoder = encoder_list[0]
        r_encoder = encoder_list[1]
        l_encoder.set_name("left_encoder") 
        r_encoder.set_name("right_encoder")
        self.l_controller.set_encoder(l_encoder)
        self.r_controller.set_encoder(r_encoder)

    def get_settings(self):
        return [self.l_controller.target * (1 if self.l_controller.get_is_fwd else -1),
                self.r_controller.target * (1 if self.r_controller.get_is_fwd else -1),
                self.l_controller.integral, self.r_controller.integral]

    def ticks(self, dt):
        left_tick_speed = self.l_encoder.poll_ticks_per_sec()
        right_tick_speed = self.r_encoder.poll_ticks_per_sec()
        left_speed, right_speed = self.signed_speed(left_tick_speed / TICKS_PER_METER,
                                                    right_tick_speed / TICKS_PER_METER)

        self.control(left_speed, right_speed, dt)
        speeds = (left_speed, right_speed)
        self.odom_publisher.publish_odom(speeds, dt, now)

    
    def control(self, left_speed, right_speed, dt):
        pwm_left, pwm_right = self.active_controller.control(left_speed, right_speed, dt)
        self.mover.set_duty_cycle(pwm_left, pwm_right)
        self.publish_PID_data(left_speed, right_speed, pwm_left, pwm_right)
        
    def publish_PID_data(self, left_speed, right_speed, pwm_left, pwm_right):
        left_target, left_i = self.l_controller.get_settings()
        right_target, right_i = self.r_controller.get_settings()
        # publishes previous pwm to match up with backward looking speed
        update_PID_msg(self.data_msg_l, self.prev_pwms[0], left_target, left_speed, 100)
        update_PID_msg(self.data_msg_r, self.prev_pwms[1], right_target, right_speed, 100)
        self.prev_pwms = (pwm_left, pwm_right)
        self.pub_l.publish(self.data_msg_l)
        self.pub_r.publish(self.data_msg_r)

    def get_motor_directions(self):
        return self.l_controller.get_is_fwd(), self.r_controller.get_is_fwd()
   
    def signed_speed(self, left_speed, right_speed):
        left_fwd = self.l_controller.get_is_fwd()
        right_fwd = self.r_controller.get_is_fwd()
        return (left_speed if left_fwd else -left_speed), \
               (right_speed if right_fwd else -right_speed)
