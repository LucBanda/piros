#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
from sensor_msgs.msg import Imu

led0=21
global pwm

def initGpio():
	gpio.setmode(gpio.BCM)


def power_on_led():
	global pwm
	gpio.setup(21, gpio.OUT)
	pwm = gpio.PWM(led0, 2) 
	pwm.start(50)

def callback(data):
	print "callback received"
	pwm.stop()
	gpio.output(led0, gpio.HIGH)
    
def listener():
	rospy.init_node('power_listener', anonymous=True)
	initGpio()

	power_on_led()
	rospy.Subscriber("/imu", Imu, callback)

	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
    
if __name__ == '__main__':
    listener()
