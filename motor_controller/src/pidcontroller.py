#!/usr/bin/env python

import sys
import rospy
import rospkg
import tf
import numpy as np
from mover import Mover
from wheelencoder import WheelEncoder
from odometry import OdometryPublisher
from geometry_msgs.msg import Twist
from motion_control.msg import PID_out, PID_settings

PKG_NAME = "motion_control"

LINE = 0.15  # M/S
TURN = 0.075  # M/S
BASE_WIDTH = 0.13  # m
# WHEEL_RADIUS = 0.0275  # m
# TICKS_PER_REVOLUTION = 36
TICKS_PER_METER = 209  # TICKS_PER_REV / (WHEEL_RADIUS * 2  * PI)
MAX_PWM = 30  # hard max to preserve motors via clamp
SPEED_AT_100_PWM = 3.0  # m/s, used to scale PID errors


def update_PID_msg(msg, pwm, target_vel, encoder_vel, scale=1):
    msg.PWM = pwm
    msg.targetVel = target_vel * scale
    msg.encoderVel = encoder_vel * scale


def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


# numpad indexing
def twist_to_index(twist):
    vec_angular, vec_linear = twist.angular, twist.linear
    if vec_angular.z > 0.2:
        turn = 1
    elif vec_angular.z < -0.2:
        turn = -1
    else:
        turn = 0
    if vec_linear.x > 0.05:
        forward = 1
    elif vec_linear.x < -0.05:
        forward = -1
    else:
        forward = 0
    return 5 - turn + 3 * forward


def twist_to_wheel_vel(twist):
    vec_angular, vec_linear = twist.angular, twist.linear
    left_vel = vec_linear.x - 0.5 * vec_angular.z * BASE_WIDTH
    right_vel = vec_linear.x + 0.5 * vec_angular.z * BASE_WIDTH
    return left_vel, right_vel


class PIDModel:
    kd = 0.0
    kp = 0.2
    ki = 1.0
    
    def __init__(self, target=0.0, integral=0.0):
        self.target = target
        self.integral = integral
        self.previous_error = 0

    def set_target(self, target):
        self.target = target

    def update(self, value, dt):
        if self.target == 0:
            self.integral = 0
            return 0
        error = (self.target - value) * 100 / SPEED_AT_100_PWM
        delta_error = error - self.previous_error
        derivative = (delta_error / dt if dt > 0 else 0)
        ret_val = self.ki * self.integral + self.kp * error + self.kd * derivative
        if self.kp != 0:  # allows disabling changes to pwm by setting kp to 0
            self.integral += error * dt
        self.previous_error = error
        return min(ret_val, MAX_PWM)


class PIDController:
    def __init__(self, left_speed=0.0, right_speed=0.0, integral_left=0.0, integral_right=0.0):
        self.left_model = PIDModel(integral=integral_left)
        self.right_model = PIDModel(integral=integral_right)
        self.reset_targets(left_speed, right_speed)

    def control(self, left_speed, right_speed, dt):
        return clamp(self.left_model.update(abs(left_speed), dt), 0, MAX_PWM), \
               clamp(self.right_model.update(abs(right_speed), dt), 0, MAX_PWM)

    def get_settings(self):
        return [self.left_model.target * (1 if self.is_left_fwd else -1),
                self.right_model.target * (1 if self.is_right_fwd else -1),
                self.left_model.integral, self.right_model.integral]

    def reset_targets(self, left_speed, right_speed):
        self.left_model.set_target(abs(left_speed))
        self.right_model.set_target(abs(right_speed))
        self.is_left_fwd = left_speed >= 0
        self.is_right_fwd = right_speed >= 0


class MotionController:
    def __init__(self, path="", single_mode=False):
        self.path = path
        self.single_mode = single_mode
        self.mover = Mover()
        self.wheel_encoder = WheelEncoder()
        self.odom_publisher = OdometryPublisher(1 / (3.14 * BASE_WIDTH), path)
        self.now = None
        self.pid_list = self.load_settings()
        self.active_controller = None
        self.set_active_controller(5)

        # initializing ROS publishers, subscribers and messages
        self.prev_pwms = (0, 0)
        rospy.Subscriber('/cmd_vel', Twist, self.move, queue_size=1)
        rospy.Subscriber("/PID_settings", PID_settings, self.set_k, queue_size=1)
        self.pub_l = rospy.Publisher("PID_l", PID_out, queue_size=10)
        self.pub_r = rospy.Publisher("PID_r", PID_out, queue_size=10)
        self.data_msg_l = PID_out(wheel="left")
        self.data_msg_r = PID_out(wheel="right")

        print("PID Master loaded")
        self.update_pwms()

    @staticmethod
    def set_k(settings):
        print("PID settings changed from ", PIDModel.kp, PIDModel.ki, PIDModel.kd,
              " to ", settings.Kp, settings.Ki, settings.Kd)
        PIDModel.kp = settings.Kp
        PIDModel.ki = settings.Ki
        PIDModel.kd = settings.Kd

    def save_settings(self):
        settings = np.array([pid.get_settings() for pid in self.pid_list])
        np.savetxt(self.path + "pid_settings", settings, fmt="%1.3f",
                   header="vel_l(m/s), vel_r(m/s), pwm_l, pwm_r")

    def load_settings(self):
        try:
            settings = np.loadtxt(self.path + "pid_settings", ndmin=2)
            pid_list = [PIDController(*pid) for pid in settings.tolist()]
            if len(pid_list) < 10:
                pid_list.extend([PIDController(0, 0) for i in range(10 - len(pid_list))])
            print("PID settings loaded")
        except:
            pid_list = [PIDController(0, 0),
                        PIDController(-LINE, -0.65*LINE, 28, 2),
                        PIDController(-LINE, -LINE, 10, 10),
                        PIDController(-0.65*LINE, -LINE, 2, 20),
                        PIDController(-TURN * 0.5, TURN * 0.5, 15, 13 ),
                        PIDController(0, 0), 
                        PIDController(TURN * 0.5, -TURN * 0.5, 14, 13),
                        PIDController(0.5 * LINE, LINE, 3, 22), 
                        PIDController(LINE, LINE, 10, 10),
                        PIDController(LINE, 0.5 * LINE, 24, 1)]
            print("No file found in:", self.path,
                  "Default PID settings loaded - will learn from scratch")
        return pid_list

    def update_pwms(self):
        rate = rospy.Rate(10)
        last_save = 0
        while not rospy.is_shutdown():
            self.ticks()
            last_save += 1
            if last_save > 600:
                self.save_settings()
                self.odom_publisher.save_weights()
                last_save = 0
            rate.sleep()

    def set_active_controller(self, index=5):
        self.active_controller = self.pid_list[index]

    def get_motor_directions(self):
        return self.active_controller.is_left_fwd, self.active_controller.is_right_fwd

    def ticks(self):
        left_tick_speed, right_tick_speed = self.wheel_encoder.poll_tick_speeds()
        left_speed, right_speed = self.signed_speed(left_tick_speed / TICKS_PER_METER,
                                                    right_tick_speed / TICKS_PER_METER)

        now = rospy.get_rostime()
        dt = ((now - self.now).to_sec() if self.now else 0)
        self.control(left_speed, right_speed, dt)
        self.odom_publisher.publish_odom(left_speed, right_speed, dt, now)
        self.now = now

    def control(self, left_speed, right_speed, dt):
        pwm_left, pwm_right = self.active_controller.control(left_speed, right_speed, dt)
        self.mover.set_duty_cycle(pwm_left, pwm_right)
        self.publish_PID_data(left_speed, right_speed, pwm_left, pwm_right)
        
    def publish_PID_data(self, left_speed, right_speed, pwm_left, pwm_right):
        left_target, right_target, left_i, right_i = self.active_controller.get_settings()
        # publishes previous pwm to match up with backward looking speed
        update_PID_msg(self.data_msg_l, self.prev_pwms[0], left_target, left_speed, 100)
        update_PID_msg(self.data_msg_r, self.prev_pwms[1], right_target, right_speed, 100)
        self.prev_pwms = (pwm_left, pwm_right)
        self.pub_l.publish(self.data_msg_l)
        self.pub_r.publish(self.data_msg_r)
    
    def move(self, twist):
        if self.single_mode:
            idx = 0
            self.pid_list[0].reset_targets(*twist_to_wheel_vel(twist))
        else:
            idx = twist_to_index(twist)
            print("Setting drive mode to ", idx)
        self.set_active_controller(idx)
        is_fwd_left, is_fwd_right = self.get_motor_directions()
        self.mover.power_motors(is_fwd_left, is_fwd_right)

    def signed_speed(self, left_speed, right_speed):
        left_fwd, right_fwd = self.get_motor_directions()
        return (left_speed if left_fwd else -left_speed), \
               (right_speed if right_fwd else -right_speed)


if __name__ == '__main__':
    try:
        rospy.init_node('pid_controller', anonymous=False)
        path_name = rospkg.RosPack().get_path(PKG_NAME) + "/src/"
        is_single = (len(sys.argv) > 1 and sys.argv[1] == "single_mode")
        MotionController(path_name, single_mode=is_single)
    except rospy.ROSInterruptException():
            pass
