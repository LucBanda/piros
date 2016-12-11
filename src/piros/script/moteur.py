#!/usr/bin/env python
import rospy
import RPi.GPIO as gpio
from std_msgs.msg import String
from geometry_msgs.msg import Twist

enA=27
enB=24
fwdA=22
fwdB=18
bckA=17
bckB=23

global enAPwm
global enBPwm
global lastTimeStamp
lastTimeStamp = None


def initGpio():
	global enAPwm
	global enBPwm
	gpio.setmode(gpio.BCM)
	gpio.setup(fwdA, gpio.OUT)
	gpio.setup(fwdB, gpio.OUT)
	gpio.setup(bckA, gpio.OUT)
	gpio.setup(bckB, gpio.OUT)
	gpio.setup(enA, gpio.OUT)
	gpio.setup(enB, gpio.OUT)
	
	enAPwm = gpio.PWM(enA, 50) # enable gpio pwm 10Hz
	enBPwm = gpio.PWM(enB, 50) # enable gpio pwm 10Hz
	print("initializing pwms");
	enBPwm.start(0)
	enAPwm.start(0)


def set_speed_left(v_l):
	global enAPwm
	gpio.output(fwdA, gpio.HIGH if (v_l > 0) else gpio.LOW)
	gpio.output(bckA, gpio.LOW if (v_l > 0) else gpio.HIGH)
	
	if (v_l > 0):
		enAPwm.ChangeDutyCycle(v_l)
	else:
		enAPwm.ChangeDutyCycle(-v_l)

def set_speed_right(v_l):
	global enBPwm
	gpio.output(fwdB, gpio.HIGH if (v_l > 0) else gpio.LOW)
	gpio.output(bckB, gpio.LOW if (v_l > 0) else gpio.HIGH)
	
	if (v_l > 0):
		enBPwm.ChangeDutyCycle(v_l)
	else:
		enBPwm.ChangeDutyCycle(-v_l)

def timeoutcheck():
	global lastTimeStamp
	if lastTimeStamp != None:
		d = rospy.Time.now() - lastTimeStamp 
		print("duration ", d.to_nsec())
		seconds = d.to_nsec() #floating point
		if seconds > 500000000:
			print("resetting GPIOs %f ; lasttimestamp = %f", seconds)
			enAPwm.ChangeDutyCycle(0)
			enBPwm.ChangeDutyCycle(0)
			lastTimeStamp = None
	
def callback(msg):
	global lastTimeStamp
	rospy.loginfo("Received a /cmd_vel message!")
	rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
	rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
	lastTimeStamp = rospy.Time.now()
	
	# Do velocity processing here:
	# Use the kinematics of your robot to map linear and angular velocities into motor commands

	v_l = msg.linear.x * 50.0
	v_r = msg.linear.x * 50.0
	v_l = v_l + msg.angular.x*50.0
	v_r = v_r - msg.angular.x*50.0
	

	# Then set your wheel speeds (using wheel_left and wheel_right as examples)
	set_speed_left(v_l)
	set_speed_right(v_r)

def listener():
	global enAPwm
	global enBPwm
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	initGpio()
	rospy.init_node('moteur', anonymous=True)


	rospy.Subscriber("/cmd_vel", Twist, callback)

	#test:
	
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		timeoutcheck()
		r.sleep()
	enAPwm.stop()
	enBPwm.stop()
	gpio.cleanup()
	
if __name__ == '__main__':
	listener()
