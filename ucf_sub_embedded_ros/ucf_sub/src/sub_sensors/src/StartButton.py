#!/usr/bin/env python3
import gpio
import rospy
import time
from std_msgs.msg import Bool

CHANNEL = 395
gpio.setup(CHANNEL, gpio.IN)

if __name__ == '__main__':
    rospy.init_node("StartButton")
    start_pub = rospy.Publisher("/start", Bool, queue_size=10)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        while gpio.input(CHANNEL):
            r.sleep()
        while not gpio.input(CHANNEL):
            r.sleep()
        start_pub.publish(Bool(True))
