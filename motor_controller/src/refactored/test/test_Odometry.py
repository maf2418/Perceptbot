#!/usr/bin/env python

import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.getcwd() + '/src')

from odometry2 import OdometryPublisher, TwoWheelOdometryCalculator
from wheelencoder2 import Encoder
from mock import MagicMock, patch
from utilities import *

class TestOdometryPublisher(unittest.TestCase):

    def setUp(self):
        self.turning_radius = 1 / (3.14 * BASE_WIDTH)
        self.testOdometryPublisher = OdometryPublisher( \
                self.turning_radius, \
                TwoWheelOdometryCalculator(2)) # 2 == number of wheels

    def testLoadWeights(self):
        expected_weights = np.loadtxt("odom_weights", ndmin=2)
        test_weights = self.testOdometryPublisher._load_weights(self.turning_radius)
        self.assertEqual(test_weights, expected_weights)

class TestOdometryCalculator(unittest.TestCase):

    def setUp(self):
        self.testOdometryCalculator = TwoWheelOdometryCalculator(2)

    def testCalcGreeks(self):
        expected_return_vals = (0, 0)
        actual_return_vals = self.testOdometryCalculator.calc_greeks( \
                [0,0], \
                [0,0,0])
        self.assertEqual(expected_return_vals, actual_return_vals)


if __name__ == '__main__':
    unittest.main()
