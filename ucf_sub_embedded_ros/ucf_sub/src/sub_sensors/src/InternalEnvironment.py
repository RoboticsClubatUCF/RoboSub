#!/usr/bin/env python
import HIH6130
import rospy
from sensor_msgs import Temperature
from sensor_msgs import RelativeHumidity

def publish():
	tempPub = rospy.Publisher('InternalTemperature', Temperature, queue_size=1)
	humidPub = rospy.Publisher('InternalHumidity', RelativeHumidity, queue_size=1)
	rospy.init_node('InternalEnvironment')
	sensor = HIH3160()
	temp = Temperature()
	humid = RelativeHumidity()
	freq = rospy.Rate(1)
	while not rospy.is_shutdown():
		sensor.read()
		temp.temperature = sensor.t
		humid.relative_humidity = sensor.h
		tempPub.publish(temp)
		humidPub.publish(humid)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
