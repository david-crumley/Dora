#!/usr/bin/env python

import RPi.GPIO as GPIO
import sys
import time
import rospy
from std_msgs.msg import Char

RightWheelDir = 22
RightWheelStp = 18
LeftWheelDir  = 15
LeftWheelStp  = 13		# Not sure if this is a GPIO

StepsPerMsg = 1
StepInterval = .002

def drive_callback(data):
	rospy.loginfo("I heard %c", data.data)

# FOR NOW: Every movement is ONE full step
	
	# Forward
	if data.data == 'w':
		# assuming DIR being High is forward
		GPIO.output(RightWheelDir, GPIO.HIGH)
		GPIO.output(LeftWheelDir, GPIO.HIGH)
		
		

	# Backward
	elif data.data == 's':
		# assuming DIR being Low is backward
		GPIO.output(RightWheelDir, GPIO.LOW)
		GPIO.output(LeftWheelDir, GPIO.LOW)
		
		

	# Rotate Right
	elif data.data == 'd':
		# assuming DIR being Low is backward
		GPIO.output(RightWheelDir, GPIO.LOW)
		GPIO.output(LeftWheelDir, GPIO.HIGH)
		
		
	#Rotate Left
	elif data.data == 'a':
		# assuming DIR being Low is backward
		GPIO.output(RightWheelDir, GPIO.HIGH)
		GPIO.output(LeftWheelDir, GPIO.LOW)

	# Moving ONE step
	GPIO.output(RightWheelStp, GPIO.HIGH)
	GPIO.output(LeftWheelStp, GPIO.HIGH)

	time.sleep(StepInterval)

	GPIO.output(RightWheelStp, GPIO.LOW)
	GPIO.output(LeftWheelStp, GPIO.LOW)
		
	time.sleep(StepInterval)



def motor_drive():

	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(RightWheelDir, GPIO.OUT)
	GPIO.setup(RightWheelStp, GPIO.OUT)
	GPIO.setup(LeftWheelDir, GPIO.OUT)
	GPIO.setup(LeftWheelStp, GPIO.OUT)	

	rospy.init_node('motor_drive_sub', anonymous=True)

	rospy.Subscriber("motor_drive", Char, drive_callback)

	rospy.spin()

	GPIO.cleanup()

if __name__ == '__main__':
	motor_drive()


