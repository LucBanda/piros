#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
from std_msgs.msg import String


def initGpio():
	gpio.setmode(gpio.BCM)


def power_on_led(gpioNb):
	gpio.setup(gpioNb, gpio.OUT)
	gpio.output(gpioNb, gpio.HIGH)

def listener():
	rospy.init_node('power_listener', anonymous=True)
	led0=21
	
	initGpio()
	power_on_led(led0)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
    
if __name__ == '__main__':
    listener()
