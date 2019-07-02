#!/usr/bin/env python

from mover import Mover
from drivechain import TwoWheelDriveChain

from utilities import *


class MotionController:
    '''
    MotionController class. Takes a DriveChain object in
    its constructor, which 
    '''
    def __init__(self, drivechain, single_mode=False, path=""):
        self.path = path
        self.single_mode = single_mode
        self.mover = Mover()
        self.now = None
        # load up DriveChains
        self.load_drivechains()
        self.active_drivechain = None
        self.set_active_drivechain(5)

        # set up ROS publishers, subscribers and messages
        rospy.Subscriber('/cmd_vel', Twist, self.move, queue_size=1)

        print("MotionController loaded")
        self.update_pwms()

    # this is the only part which (sadly) knows about TwoWheelDriveChain
    def load_drivechains(self):
        try:
            # TODO rename this file "drivechain_settings"
            settings = np.loadtxt(self.path + "pid_settings", ndmin=2)
            drivechain_list = [TwoWheelDriveChain(*dc) for dc in settings.tolist()]
            if len(drivechain_list) < 10:
                drivechain_list.extend([TwoWheelDriveChain() for i in range(10 - len(drivechain_list))])
                print("Rock and roll!! DriveChain's loaded :D")
        except:
            drivechain_list = [TwoWheelDriveChain(0, 0),
                        TwoWheelDriveChain(-LINE, -0.5*LINE, 28, 2),
                        TwoWheelDriveChain(-LINE, -LINE, 10, 10),
                        TwoWheelDriveChain(-0.5*LINE, -LINE, 2, 20),
                        TwoWheelDriveChain(-TURN * 0.5, TURN * 0.5, 15, 13 ),
                        TwoWheelDriveChain(0, 0), 
                        TwoWheelDriveChain(TURN * 0.5, -TURN * 0.5, 14, 13),
                        TwoWheelDriveChain(0.5 * LINE, LINE, 3, 22), 
                        TwoWheelDriveChain(LINE, LINE, 10, 10),
                        TwoWheelDriveChain(LINE, 0.5 * LINE, 24, 1)]
            print("No file found in:", self.path,
                  "Default PID settings loaded - will learn from scratch")
        return drivechain_list
    
    # TODO rename this file "drivechain_settings"
    def save_settings(self):
        settings = np.array([dc.get_settings() for dc in self.drivechain_list])
        np.savetxt(self.path + "pid_settings", settings, fmt="%1.3f",
                   header="vel_l(m/s), vel_r(m/s), pwm_l, pwm_r")

    def update_pwms(self):
        rate = rospy.Rate(10)
        last_save = 0
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            dt = ((now - self.now).to_sec() if self.now else 0)
            self.active_drivechain.ticks(dt) # call our DriveChain to update itself
            last_save += 1
            if last_save > 600:
                self.save_settings()
                self.odom_publisher.save_weights()
                last_save = 0
            self.now = now
            rate.sleep()

    def set_active_drivechain(self, index=5):
        self.active_drivechain = self.drivechain_list[index]

    def move(self, twist):
        if self.single_mode:
            idx = 0
            self.drivechain_list[0].reset_targets(*twist_to_wheel_vel(twist))
        else:
            idx = twist_to_index(twist)
            print("Setting drive mode to ", idx)
        self.set_active_drivechain(idx)
        is_fwd_left, is_fwd_right = self.active_drivechain.get_motor_directions()
        self.mover.power_motors(is_fwd_left, is_fwd_right)

