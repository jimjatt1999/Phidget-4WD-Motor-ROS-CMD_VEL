#!/usr/bin/env python2.7
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
from geometry_msgs.msg import Twist
import time
import rospy

lf_motor = BLDCMotor() #LEFT FRONT MOTOR
rf_motor = BLDCMotor() #RIGHT FRONT MOTOR
lr_motor = BLDCMotor() #LEFT REAR MOTOR
rr_motor = BLDCMotor() #REAR RIGHT MOTOR

def connect_motor(motor, name):
	status = False
	while(status == False):
		status = True
		try:
			motor.openWaitForAttachment(5000)
		except:
			status = False
			print("Failed to connect " + name + " Motor. Trying again...")
			time.sleep(1)

def init_motor(motor):
	motor.setTargetVelocity(0)
	motor.setAcceleration(80)
	motor.setFanMode(FanMode.FAN_MODE_AUTO)

def bound(min_val, max_val, value):
	return max(min_val, min(max_val, value))

def callback(data):
	left_targetVelocity = bound(-1, 1, (data.linear.x + data.angular.z))
	right_targetVelocity = bound(-1, 1, (data.linear.x - data.angular.z))
	lf_motor.setTargetVelocity(left_targetVelocity)
	rf_motor.setTargetVelocity(left_targetVelocity)
	lr_motor.setTargetVelocity(right_targetVelocity)
	rr_motor.setTargetVelocity(right_targetVelocity)

def main():
	rospy.init_node('mecanum_drive', anonymous=True)

	rospy.Subscriber("/cmd_vel", Twist, callback)

	lf_motor.setHubPort(1)
	rf_motor.setHubPort(2)
	lr_motor.setHubPort(4)
	rr_motor.setHubPort(3)

	connect_motor(lf_motor, "LF")
	connect_motor(rf_motor, "RF")
	connect_motor(lr_motor, "LR")
	connect_motor(rr_motor, "RR")

	init_motor(lf_motor)
	init_motor(rf_motor)
	init_motor(lr_motor)
	init_motor(rr_motor)
	print("Motors Ready")

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		lf_motor.close()
		rf_motor.close()
		lr_motor.close()
		rr_motor.close()
		pass