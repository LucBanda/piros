#!/usr/bin/env python

import RPi.GPIO as gpio
import rospy
from nav_msgs.msg import Path

led0=21
global pwm
global lastTimeStamp

def initGpio():
	gpio.setmode(gpio.BCM)


def blinking_led():
	global pwm
	gpio.setup(21, gpio.OUT)
	pwm = gpio.PWM(led0, 2) 
	pwm.start(50)

def callback(data):
	global pwm
	global lastTimeStamp
	lastTimeStamp = rospy.Time.now()
	if pwm != None:
		pwm.stop()
		pwm = None
		gpio.setup(21, gpio.OUT)
	gpio.output(led0, gpio.HIGH)
	
def timeoutcheck():
	global lastTimeStamp
	if lastTimeStamp != None:
		d = rospy.Time.now() - lastTimeStamp 
		seconds = d.to_nsec() #floating point
		if seconds > 50000000:
			blinking_led()
			lastTimeStamp = None
			
def listener():
	global lastTimeStamp
	rospy.init_node('power_listener', anonymous=True)
	initGpio()

	blinking_led()
	lastTimeStamp = None
	rospy.Subscriber("/path", Path, callback)

	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		timeoutcheck()
		r.sleep()
	pwm.stop()
	gpio.cleanup()
    
if __name__ == '__main__':
    listener()
