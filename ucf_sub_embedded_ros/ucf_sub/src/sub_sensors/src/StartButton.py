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
    r = rospy.Rate(4)
    while not rospy.is_shutdown():
        if not gpio.input(CHANNEL):
            start_pub.publish(Bool(True))
        r.sleep()
