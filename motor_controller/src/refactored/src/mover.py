#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy

from geometry_msgs.msg import Twist

# global constants (numbers are BOARD numberings)
# speeds are % of duty cycle

PWM_LEFT = 33
PWM_RIGHT = 12
GPIO_MOTOR1 = 11
GPIO_MOTOR2 = 15
GPIO_MOTOR3 = 16
GPIO_MOTOR4 = 18
PWM_FREQ = 25

class Mover(object):
    # it's not possible for us to test __init__ magic methods :(
    def __init__(self, debug=False):
        self.debug = debug
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(GPIO_MOTOR1, GPIO.OUT)
        GPIO.setup(GPIO_MOTOR2, GPIO.OUT)
        GPIO.setup(GPIO_MOTOR3, GPIO.OUT)
        GPIO.setup(GPIO_MOTOR4, GPIO.OUT)
        # PWM pins
        GPIO.setup(PWM_LEFT,GPIO.OUT)  
        GPIO.setup(PWM_RIGHT,GPIO.OUT)
        self.pwm0 = GPIO.PWM(PWM_RIGHT,PWM_FREQ) # channel, frequency
        self.pwm1 = GPIO.PWM(PWM_LEFT,PWM_FREQ)
        self.start_pwm()
        self.set_duty_cycle(0, 0)

    # tested in gpio_test.py
    def start_pwm(self):
        self.pwm0.start(100)  # use 100% of duty cycle
        self.pwm1.start(100)

    def power_motors(self, is_forward_l, is_forward_r):
        GPIO.output(GPIO_MOTOR1, not is_forward_r)
        GPIO.output(GPIO_MOTOR2, is_forward_r)
        GPIO.output(GPIO_MOTOR3, not is_forward_l)
        GPIO.output(GPIO_MOTOR4, is_forward_l)

    def set_duty_cycle(self, pwm_left, pwm_right):  # turns in the CURRENT direction
        self.pwm0.ChangeDutyCycle(abs(pwm_right))
        self.pwm1.ChangeDutyCycle(abs(pwm_left))
        
    # tested in gpio_test.py
    def stop(self):
        self.pwm0.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(0)

    # tested in gpio_test.py
    def teardown(self):
        self.pwm0.stop()
        self.pwm1.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        rospy.init_node('mover', anonymous=False)
        Mover()
    except rospy.ROSInterruptException():
        pass
