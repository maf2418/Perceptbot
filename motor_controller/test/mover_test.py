#!/usr/bin/env python

import RPi.GPIO as GPIO
import unittest
import geometry_msgs.msg
import sys
import os

#sys.path.append('../')  # necessary hack to import mover class
sys.path.insert(0, os.getcwd() + '/src')

from mock import patch, call, Mock, MagicMock
from mover import Mover
from geometry_msgs.msg import Vector3

# I think that decorating the test class with @patch in this way
# ensures that all test methods have access to a mocked instance
# of each class...

@patch('mover.Mover', autospec=True)
class TestGenericMoverMethods(unittest.TestCase):

	PWM_LEFT = 33
	PWM_RIGHT = 12
	GPIO_MOTOR1 = 11
	GPIO_MOTOR2 = 15
	GPIO_MOTOR3 = 16
	GPIO_MOTOR4 = 18

        GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(GPIO_MOTOR1, GPIO.OUT)
	GPIO.setup(GPIO_MOTOR2, GPIO.OUT)
	GPIO.setup(GPIO_MOTOR3, GPIO.OUT)
	GPIO.setup(GPIO_MOTOR4, GPIO.OUT)
	# PWM pins
	GPIO.setup(PWM_LEFT,GPIO.OUT)  
	GPIO.setup(PWM_RIGHT,GPIO.OUT)
        
        # TODO
	def test_moverSetup(self, MockMover):
		mover = MockMover()
		mover.forward()
		mover.assert_has_calls([call(self.GPIO_MOTOR1, False),
								call(self.GPIO_MOTOR2, True),
								call(self.GPIO_MOTOR3, False),
								call(self.GPIO_MOTOR4, True)])


class TestMoverVelocityCalculations(unittest.TestCase):

    mover = Mover(debug=True)

    def createTwistVectors(self, linearX, angularZ):
        angular = Vector3(0,0,angularZ)
        linear = Vector3(linearX,0,0)
        return linear, angular

    def test_velocity00(self):
        self._test_velocityCalculation([0,0,0,0])

    def test_velocity01(self):
        self._test_velocityCalculation([1,0,1,1])

    def test_velocity02(self):
        self._test_velocityCalculation([-1,0,-1,-1])
    
    def test_velocity03(self):
        self._test_velocityCalculation([-1,-1,-1.07,-0.93])
    
    def test_velocity04(self):
        self._test_velocityCalculation([1,1,1.07,0.93])
    
    def test_velocity05(self):
        self._test_velocityCalculation([1,-1,0.93,1.07])
    
    def test_velocity06(self):
        self._test_velocityCalculation([-1,0.5,-0.965,-1.035])
    
    def test_velocity07(self):
        self._test_velocityCalculation([0.5,-1,0.43,0.57])

    def _test_velocityCalculation(self, params):
        lin, ang = self.createTwistVectors(params[0],params[1])
        velL, velR = self.mover.calculate_velocity(lin, ang)
        self.assertEqual(velL,params[2])
        self.assertEqual(velR,params[3])

    def test_unpackTwist(self):
    	# construct a dummy msg to unpack
    	twist = geometry_msgs.msg.Twist()
    	twist.linear.x = 0.5
    	twist.angular.z = 1
    	linear, angular = self.mover.unpack_twist(twist)  # unpack_twist returns SOME value
    	self.assertEqual(linear.x, 0.5)  # assert value for linear component
    	self.assertEqual(angular.z, 1)  # assert value for angular component


if __name__ == '__main__':
	unittest.main()

#suite = unittest.TestLoader().loadTestsFromTestCase(TestMoverVelocityCalculations)
#unittest.TextTestRunner(verbosity=3).run(suite)
#suite = unittest.TestLoader().loadTestsFromTestCase(TestGenericMoverMethods)
#unittest.TextTestRunner(verbosity=3).run(suite)
