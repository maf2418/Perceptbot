#!/usr/bin/env python

import os
import sys
import unittest

sys.path.insert(0, os.getcwd() + '/src')

from drivechain import TwoWheelDriveChain
from wheelencoder2 import Encoder
from mock import MagicMock, patch

class TestTwoWheelDriveChain(unittest.TestCase):
    
    def setUp(self):
        self.testDriveChain = TwoWheelDriveChain()

    def testLoadingEncoderParamsFromFile(self):
        self.testDriveChain.load_encoder_settings()
        self.assertEqual(self.testDriveChain.l_encoder_pin, 38)
        self.assertEqual(self.testDriveChain.r_encoder_pin, 40)

    def testCanPublishOdom(self):
        self.fail()

    def testCanSetActivePID(self):
        self.fail()

    # TODO
    def testGetSettings(self):
        # set some dummy vals
        self.testDriveChain.l_controller.target = 0
        self.testDriveChain.r_controller.target = 0
        self.testDriveChain.l_controller.is_fwd = True
        self.testDriveChain.r_controller.is_fwd = True
        self.testDriveChain.l_controller.integral = 0
        self.testDriveChain.r_controller.integral = 0
        actual_return_val = self.testDriveChain.get_settings()
        expected_return_val = [0, 0, 0, 0]
        self.assertEqual(actual_return_val, expected_return_val)

    def testControl(self):
        # TODO check calls to mock publisher, mover objects
        self.fail()

    def testPublishPIDdata(self):
        # TODO check calls to mock publisher object
        self.fail()

    def testGetMotorDirections(self):
        self.testDriveChain.l_controller.is_fwd = True
        self.testDriveChain.r_controller.is_fwd = True
        expected_return_val = (True, True)
        actual_return_val = self.testDriveChain.get_motor_directions()
        self.assertEqual(expected_return_val, actual_return_val)

    def testSignedSpeed(self):
        self.testDriveChain.l_controller.is_fwd = True
        self.testDriveChain.r_controller.is_fwd = True
        expected_return_val = (1, 1)
        actual_return_val = self.testDriveChain.signed_speed(1, 1)
        self.assertEqual(expected_return_val, actual_return_val)

if __name__ == '__main__':
    unittest.main()
