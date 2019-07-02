#!/usr/bin/env python

import os
import sys
import unittest

sys.path.insert(0, os.getcwd() + '/src')

from utilities import *

class DummyPIDMessage:

    def __init__(self, wheel, pwm, target_vel, encoder_vel):
        self.PWM = pwm
        self.targetVel = target_vel
        self.encoderVel = encoder_vel

class DummyTwist:

    def __init__(self, vec_ang, vec_lin, x, z):
        self.angular = DummyTwistElement(z)
        self.linear = DummyTwistElement(x)

class DummyTwistElement:

    def __init__(self, x=0.0, z=0.0):
        self.x = x
        self.z = z

class TestUtilities(unittest.TestCase):

    def testUpdatePIDmsg(self):
        test_message = DummyPIDMessage("left", 10.0, 10.0, 10.0)
        update_PID_msg(test_message, 15.0, 15.0, 15.0)
        self.assertEqual(test_message.PWM, 15.0)
        self.assertEqual(test_message.targetVel, 15.0)
        self.assertEqual(test_message.encoderVel, 15.0)

    def testClamp(self):
        test_val = 5
        expected_return_val = clamp(test_val, 2, 50)
        self.assertEqual(expected_return_val, test_val)

    def testTwistToIndex(self):
        twist = DummyTwist(1, 1, 0.1, 0.1)
        actual_return_val = twist_to_index(twist)
        expected_return_val = 8
        self.assertEqual(expected_return_val, actual_return_val)

    def testTwistToWheelVel(self):
        twist = DummyTwist(1, 1, 0.1, 0.1)
        actual_return_val_l, actual_return_val_r = twist_to_wheel_vel(twist)
        #expected_return_val = (0.0935, 0.10650000000000001)
        expected_return_val = (0.1, 0.1)
        self.assertEqual((actual_return_val_l, actual_return_val_r), \
                expected_return_val)



if __name__ == '__main__':
    unittest.main()
