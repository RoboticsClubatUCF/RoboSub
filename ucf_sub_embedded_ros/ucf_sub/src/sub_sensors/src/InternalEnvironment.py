#!/usr/bin/env python3
import HIH6130
import rospy
from sensor_msgs.msg import Temperature, RelativeHumidity

def publish():
	tempPub = rospy.Publisher('InternalTemperature', Temperature, queue_size=1)
	humidPub = rospy.Publisher('InternalHumidity', RelativeHumidity, queue_size=1)
	rospy.init_node('InternalEnvironment')
	sensor = HIH6130.HIH6130(bus=1)
	temp = Temperature()
	temp.header.frame_id = "base_link"
	humid = RelativeHumidity()
	humid.header.frame_id = "base_link"
	freq = rospy.Rate(5)
	while not rospy.is_shutdown():
		sensor.read()
		temp.temperature = sensor.t
		humid.relative_humidity = sensor.rh
		tempPub.publish(temp)
		humidPub.publish(humid)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
