#!/usr/bin/env python

import sys
import rospy
import rospkg
import tf
import numpy as np
from mover import Mover
from geometry_msgs.msg import Twist
from motion_control.msg import PID_out, PID_settings
from utilities import *

PKG_NAME = "motion_control"


class PIDController:
    '''
    New PIDController object. An Encoder object is passed
    in the constructor, because only the DriveChainLayout
    knows which encoder (i.e. GPIO pins) correspond to which
    PIDController.
    '''
    kd = 0.0
    kp = 0.2
    ki = 1.0

    def __init__(self, speed=0.0, integral_speed=0.0, target=0.0, integral_target=0.0, encoder=None, name=""):
        self.target = target
        self.integral = integral_target
        self.previous_error = 0

        self.encoder = encoder

    def update(self, value, dt):
        if self.target == 0:
            self.integral_target = 0
            return 0
        error = (self.target - value) * 100 / SPEED_AT_100_PWM
        delta_error = error - self.previous_error
        derivative = (delta_error / dt if dt > 0 else 0)
        ret_val = self.ki * self.integral + self.kp * error + self.kd * derivative
        if self.kp != 0:  # allows disabling changes to pwm by setting kp to 0
            self.integral += error * dt
        self.previous_error = error
        return min(ret_val, MAX_PWM)

    def control(self, speed, dt):
        return clamp(self.update(abs(speed), dt), 0, MAX_PWM)

    def get_settings(self):
        return self.target * (1 if self.is_fwd else -1), self.integral

    def set_target(self, target, speed):
        self.target = target
        self.is_fwd = speed >= 0

    def set_encoder(self, encoder):
        self.encoder = encoder


if __name__ == '__main__':
    try:
        rospy.init_node('pid_controller', anonymous=False)
        path_name = rospkg.RosPack().get_path(PKG_NAME) + "/src/"
        is_single = (len(sys.argv) > 1 and sys.argv[1] == "single_mode")
        MotionController(path_name, single_mode=is_single)
    except rospy.ROSInterruptException():
            pass
