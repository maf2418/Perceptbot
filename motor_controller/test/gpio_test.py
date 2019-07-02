#!/usr/bin/env python

import RPi.GPIO as GPIO
import unittest
import sys
import os

#sys.path.append('../')  # necessary hack to import mover class
sys.path.insert(0, os.getcwd() + '/src')

from mock import patch, call, Mock, MagicMock
from mover import Mover

# I think that decorating the test class with @patch in this way
# ensures that all test methods have access to a mocked instance
# of each class...

@patch('RPi.GPIO.output', autospec=True)
class TestMoverMethods(unittest.TestCase):

	PWM_LEFT = 33
	PWM_RIGHT = 12
	GPIO_MOTOR1 = 11
	GPIO_MOTOR2 = 15
	GPIO_MOTOR3 = 16
	GPIO_MOTOR4 = 18

	testMover = Mover(debug=True)

        # setting up the Mover() object calls the GPIO (in some way)
	def test_moverSetup(self, MockGPIO):
		gpio = MockGPIO()
		assert MockGPIO.called  # basic assertion

	def test_startPWM(self, MockGPIO):
		gpio = MockGPIO(side_effect=RuntimeError(
					'A PWM object already exists for this GPIO channel'))
		self.testMover.start_pwm()

        def test_stop(self, MockGPIO):
                gpio = MockGPIO()
                self.testMover.stop()
                MockGPIO.assert_has_calls([call()])

	def test_moveForward(self, MockGPIO):
		self.testMover.forward()
		MockGPIO.assert_has_calls([call(self.GPIO_MOTOR1, False),
								call(self.GPIO_MOTOR2, True),
								call(self.GPIO_MOTOR3, False),
								call(self.GPIO_MOTOR4, True)])
	
	def test_moveBackward(self, MockGPIO):
		self.testMover.reverse()
		MockGPIO.assert_has_calls([call(self.GPIO_MOTOR1, True),
								call(self.GPIO_MOTOR2, False),
								call(self.GPIO_MOTOR3, True),
								call(self.GPIO_MOTOR4, False)])
        def test_teardown(self, MockGPIO):
            gpio = MockGPIO()
            self.testMover.teardown()
            MockGPIO.assert_has_calls([call()])  # this is the best we can do, test that it was called
	
	@classmethod
	def tearDownClass(self):
		GPIO.cleanup()


if __name__ == '__main__':
	unittest.main()
#suite = unittest.TestLoader().loadTestsFromTestCase(TestMoverMethods)
#unittest.TextTestRunner(verbosity=3).run(suite)
