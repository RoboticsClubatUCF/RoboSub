#!/usr/bin/env python
import ms5837
import rospy
from sensor_msgs import Temperature
import std_msgs import Float32

def publish():
	sensor = ms5837.MS5837_30BA()
	sensor.init()
	tempPub = rospy.Publisher('ExternalTemperature', Temperature, queue_size=1)
	depthPub = rospy.Publisher('Depth', Float32, queue_size=1)
	rospy.init_node('Depth')
	temp = Temperature()
	depth = Float32()
	freq = rospy.Rate(1)
	while not rospy.is_shutdown():
		sensor.read()
		temp.temperature = sensor.temperature()
		depth.data = sensor.depth()
		tempPub.publish(temp)
		depthPub.publish(depth)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
