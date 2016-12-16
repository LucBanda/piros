#!/usr/bin/env python
import rospy
import RPi.GPIO as gpio
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu





class Motor:
	enA=27
	enB=24
	fwdA=22
	fwdB=18
	bckA=17
	bckB=23
	
	def __init__(self):
		gpio.setmode(gpio.BCM)
		gpio.setup(self.fwdA, gpio.OUT)
		gpio.setup(self.fwdB, gpio.OUT)
		gpio.setup(self.bckA, gpio.OUT)
		gpio.setup(self.bckB, gpio.OUT)
		gpio.setup(self.enA, gpio.OUT)
		gpio.setup(self.enB, gpio.OUT)
	
		self.enAPwm = gpio.PWM(self.enA, 20) # enable gpio pwm 10Hz
		self.enBPwm = gpio.PWM(self.enB, 20) # enable gpio pwm 10Hz
		self.enBPwm.start(0)
		self.enAPwm.start(0)
		
	def set_speed(self, v_l, v_r):
		#set gear for left wheel
		gpio.output(self.fwdA, gpio.HIGH if (v_l > 0) else gpio.LOW)
		gpio.output(self.bckA, gpio.LOW if (v_l > 0) else gpio.HIGH)
		
		#set gear for right wheel
		gpio.output(self.fwdB, gpio.HIGH if (v_r > 0) else gpio.LOW)
		gpio.output(self.bckB, gpio.LOW if (v_r > 0) else gpio.HIGH)
		
		if (v_l < 0):
			v_l = -v_l
		
		if (v_r < 0):
			v_r = -v_r
		
		#apply command on both wheels
		self.enAPwm.ChangeDutyCycle(v_l)
		self.enBPwm.ChangeDutyCycle(v_r)
		
	def stop(self):
		self.enAPwm.stop()
		self.enBPwm.stop()
		gpio.cleanup()

	
class Autopilot:
	def __init__(self, motor_callback):
		self.motor_callback = motor_callback
		self.v_r = 0.0
		self.v_l = 0.0
		self.cmd = None
		self.imu = None
		self.lastTimeStamp = None
		
	def set_cmd(self, cmd):
		self.lastTimeStamp = rospy.Time.now()
		self.cmd = cmd
			
	def set_imu(self, imu):
		self.imu = imu
	
	def timeoutcheck(self):

		if self.lastTimeStamp != None:
			d = rospy.Time.now() - self.lastTimeStamp 
			seconds = d.to_nsec() #floating point
			if seconds > 500000000:
				self.lastTimeStamp = None
				return True
		return False

			
	def run(self):
		if (self.timeoutcheck()):
			self.motor_callback(0.0, 0.0)
			self.cmd = None
			
		if (self.cmd != None):
			v_l = self.cmd.linear.x*60
			v_r = self.cmd.linear.x*60
			v_l = v_l - self.cmd.angular.z*40
			v_r = v_r + self.cmd.angular.z*40
			self.motor_callback(v_l, v_r)


def cmd_callback(autopilot, cmd):
	autopilot.set_cmd(cmd)

def imu_callback(autopilot, imu):
	autopilot.set_imu(imu)

def listener():
	rospy.init_node('moteur', anonymous=True)

	motors = Motor()
	autopilot = Autopilot(motors.set_speed)
	
	rospy.Subscriber("/cmd_vel", Twist, autopilot.set_cmd)
	rospy.Subscriber("/imu", Imu, lambda x: autopilot.set_imu)
	
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		autopilot.run()
	
	print "stoping nicely"
	motors.stop()
	
if __name__ == '__main__':
	listener()
