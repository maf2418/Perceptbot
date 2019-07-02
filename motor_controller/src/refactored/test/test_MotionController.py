#!/usr/bin/env python

import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.getcwd() + '/src')

from motioncontroller import MotionController
from drivechain import TwoWheelDriveChain
from mover import Mover
from mock import MagicMock, patch
from utilities import *


class TestMotionController(unittest.TestCase):

    @patch('mover.Mover', autospec=True)
    @patch('drivechain.TwoWheelDriveChain', autospec=True)
    def setUp(self, MockMover, MockDriveChain):
        self.testDriveChain = MockDriveChain
        testMover = MockMover
        self.testMotionController = MotionController(self.testDriveChain)
        self.testMotionController.mover = testMover

    def testLoadDriveChain(self):
        actual_return_val = self.testMotionController.load_drivechains()
        self.testDriveChain.assert_called_with()
        self.assertEqual(len(actual_return_val), 2)

    def testSaveSettings(self):
        self.fail()

    def testUpdatePWMS(self):
        self.fail()

    def testCanSetActiveDrivechain(self):
        self.testMotionController.set_active_drivechain()
        self.assertTrue(self.testMotionController.active_drivechain)

    def testCallingMoveCallsMover(self):
#        dummyTwist = Twist()
        self.testMotionController.move(dummyTwist)
        self.testMotionController.mover.assert_called_with()


if __name__ == '__main__':
    unittest.main()
