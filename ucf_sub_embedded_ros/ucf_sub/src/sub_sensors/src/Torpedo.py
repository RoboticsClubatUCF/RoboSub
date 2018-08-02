#!/usr/bin/env python3
import gpio
import rospy
import time
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

CHANNEL1 = 394
CHANNEL2 = 393
gpio.setup(CHANNEL1, gpio.OUT)
gpio.setup(CHANNEL2, gpio.OUT)

FIRETIME = 0.04

t1Last = False
t2Last = False

def fireTorp1(data):
	global t1Last
	if data.data and not t1Last:
		gpio.output(CHANNEL1, True)
		time.sleep(FIRETIME)
		gpio.output(CHANNEL1, False)
	t1Last = data.data

def fireTorp2(data):
	global t2Last
	if data.data and not t2Last:
		gpio.output(CHANNEL2, True)
		time.sleep(FIRETIME)
		gpio.output(CHANNEL2, False)
	t2Last = data.data

def start():
	rospy.init_node('TorpedoWatcher', anonymous=True)
	rospy.Subscriber('Torpedo1', Bool, fireTorp1)
	rospy.Subscriber('Torpedo2', Bool, fireTorp2)
	rospy.spin()

if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass
