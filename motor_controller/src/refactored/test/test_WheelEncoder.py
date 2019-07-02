#!/usr/bin/env python

import os
import sys
import unittest
import RPi.GPIO as GPIO

sys.path.insert(0, os.getcwd() + '/src')

from wheelencoder2 import Encoder
from mock import patch, call, Mock, MagicMock

class TestEncoder(unittest.TestCase):

    @patch('RPi.GPIO.output', autospec=True)
    def setUp(self, MockGPIO):
        self.testEncoder = Encoder()
        gpio = MockGPIO()
        assert MockGPIO.called

    def testGetName(self):
        self.testEncoder.name = "left"
        return_val = self.testEncoder.getName()
        self.assertEqual("left", return_val)

    def testSetName(self):
        self.testEncoder.set_name("blob")
        actual_return_val = self.testEncoder.get_name()
        self.assertEqual(actual_return_val, "blob")

    def testPollTicksPerSec(self):
        # this is hard to test
        self.fail()

    def testBlockingPublish(self):
        # this is hard to test
        self.fail()


    @patch('RPi.GPIO.output', autospec=True)
    def testSettingSideSetsGPIOPins(self, MockGPIO):
        self.testEncoder.setGPIOPin(5)
        gpio = MockGPIO(side_effect=RuntimeError(
            'Conflicting edge detection already enabled for this GPIO channel'))
        self.testEncoder.setGPIOPin(5)

if __name__ == '__main__':
	unittest.main()
