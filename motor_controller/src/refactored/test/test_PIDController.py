#!/usr/bin/env python

import os
import sys
import unittest

sys.path.insert(0, os.getcwd() + '/src')

from pidcontroller2 import PIDController
from wheelencoder2 import Encoder
from mock import MagicMock, patch
from utilities import *

class TestPIDController(unittest.TestCase):
    
    def setUp(self):
        self.testPIDController = PIDController()

    def testCallingUpdateMethodSetsIntegral_1(self):
        self.testPIDController.target = 0
        expected_return_val = 0
        test_return_val = self.testPIDController.update(0,0)
        self.assertEqual(expected_return_val, test_return_val)

    def testCallingUpdateMethodSetsIntegral_2(self):
        self.testPIDController.target = 10  # some non-0 value
        expected_prev_error = (10-0) * 100 / SPEED_AT_100_PWM
        test_return_val = self.testPIDController.update(0,0)
        derivative = 0
        expected_return_val = self.testPIDController.ki * self.testPIDController.integral \
                + self.testPIDController.kp * expected_prev_error + self.testPIDController.kd * derivative
        self.assertEqual(self.testPIDController.previous_error, expected_prev_error)
        self.assertEqual(test_return_val, MAX_PWM)

    def testCallingUpdateMethodSetsIntegral_3(self):
        self.testPIDController.target = 1  # some (lower) non-0 value
        expected_prev_error = 1 * 100 / SPEED_AT_100_PWM
        test_return_val = self.testPIDController.update(0,0)
        derivative = 0
        expected_return_val = self.testPIDController.ki * self.testPIDController.integral \
                + self.testPIDController.kp * expected_prev_error + self.testPIDController.kd * derivative
        self.assertEqual(self.testPIDController.previous_error, expected_prev_error)
        self.assertEqual(test_return_val, expected_return_val)

    def testSetTarget(self):
        self.testPIDController.target = 1
        self.testPIDController.set_target(10, 45)
        self.assertEqual(self.testPIDController.target, 10)
        self.assertTrue(self.testPIDController.is_fwd)

    def testGetSettingsReturnsSettings(self):
        self.testPIDController.target = 5
        self.testPIDController.integral = 5
        self.testPIDController.is_fwd = True
        expected_return_val = (5,5) # dummy vals
        test_return_val = self.testPIDController.get_settings()
        self.assertEqual(test_return_val, expected_return_val)

    
    # TODO
    @patch('wheelencoder2.Encoder', autospec=True)
    def testCallingControlMethodPollsWheelEncoder(self, MockWheelEncoder):
        self.encoder = MockWheelEncoder


if __name__ == '__main__':
    unittest.main()
