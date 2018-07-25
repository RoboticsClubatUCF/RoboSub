#!/usr/bin/env python
#import RPi.GPIO as GPIO
import gpio
import rospy
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

CHANNEL = 429 #should be 17

def publish():
	rospy.init_node('Leak')
	leakPub = rospy.Publisher('/leak', Bool, queue_size=1)
	diagPub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
	#GPIO.setmode(GPIO.BCM)
	#GPIO.setup(CHANNEL, GPIO.IN)
	gpio.setup(CHANNEL, gpio.IN)
	freq = rospy.Rate(1)
	leak = Bool()
	diag = DiagnosticArray()
	while not rospy.is_shutdown():
		levelData=0
		leak.data = gpio.input(CHANNEL)
		if (leak.data):
			levelData=2
		diag.status = [DiagnosticStatus(name='Leak', message=str(leak.data), level=levelData)]
		leakPub.publish(leak)
		diagPub.publish(diag)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
