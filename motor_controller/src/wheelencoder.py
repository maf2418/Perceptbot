#!/usr/bin/env python

import sys
import time
import RPi.GPIO as GPIO
import rospy

from std_msgs.msg import Int32MultiArray

MOTORENCODERRIGHT = 38  # GPIO pins
MOTORENCODERLEFT = 40


class Encoder:
    def __init__(self):
        self.total_ticks = 0
        self.time_tick = None
        self.time_poll = None

    def tick(self, pin=None):
        self.total_ticks += 1
        self.time_tick = rospy.get_rostime()
        #print("tick")

    def poll_ticks_per_sec(self):
        now = rospy.get_rostime()
        dt = ((self.time_tick - self.time_poll).to_sec() if self.time_poll else 0)
        if dt > 0.001:
            speed = self.total_ticks / dt
            # caps speed for case of tick only received at beginning of period
            max_speed = (1 / (now - self.time_tick).to_sec() if now > self.time_tick else speed)
            speed = min(speed, max_speed)
            self.time_poll = self.time_tick
        else:
            speed = 0
            self.time_poll = now
            self.time_tick = now
        self.total_ticks = 0
        return speed


class WheelEncoder:
    def __init__(self):
        self.wheel_left = Encoder()
        self.wheel_right = Encoder()
        self.array_to_publish = Int32MultiArray(data=[0, 0])

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(MOTORENCODERRIGHT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(MOTORENCODERRIGHT, GPIO.RISING,
                              callback=self.wheel_right.tick, bouncetime=2)
        GPIO.setup(MOTORENCODERLEFT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(MOTORENCODERLEFT, GPIO.RISING,
                              callback=self.wheel_left.tick, bouncetime=2)

    def poll_tick_speeds(self):
        return self.wheel_left.poll_ticks_per_sec(), self.wheel_right.poll_ticks_per_sec()

    def blocking_publish(self):
        wheel_pub = rospy.Publisher('/wheel_encoders', Int32MultiArray, queue_size=10)
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.array_to_publish.data = [self.wheel_left.total_ticks, self.wheel_right.total_ticks]
            wheel_pub.publish(self.array_to_publish)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('wheel_odom', anonymous=False)
        WheelEncoder().blocking_publish()
    except rospy.ROSInterruptException():
        pass
