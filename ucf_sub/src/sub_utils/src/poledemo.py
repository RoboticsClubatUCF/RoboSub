#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from sub_utils.msg import CameraPosition
import distance_to_camera as distance


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher(
            "/contoured_image", Image, queue_size=10)
        self.lower = np.array([9, 27, 100], dtype="uint8")
        self.upper = np.array([39, 83, 247], dtype="uint8")        
	self.bridge = CvBridge()
        self.focalLength = None
        self.image_sub = rospy.Subscriber(
            "/stereo/left/image_raw", Image, self.callback)
        self.camera_position = rospy.Publisher(
            "/camera_position", CameraPosition, queue_size=10)

    def getThresholds(self, data):
        self.upper = data[0:3]
        self.lower = data[3:6]

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
	    print(e)
        #imageHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #height, width = cv_image.shape[:2]
        #res = cv2.resize(cv_image, None, fx=0.5 * width, fy=0.5 * height, interpolation=cv2.INTER_CUBIC)
        mask = cv2.inRange(cv_image, self.lower, self.upper)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_LIST,
                                cv2.CHAIN_APPROX_SIMPLE)

        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        cnt = cnts[1]
        cX = 0
        cY = 0

        maxLength = 0
        pole = []
        for c in cnt:
            	M = cv2.moments(c)
		if len(c) > maxLength:
                	try:
                    		cX = int(M["m10"] / M["m00"])
                    		cY = int(M["m01"] / M["m00"])
     			except:
                    		pass
                	maxLength = len(c)
                	pole = np.array(c)
	print(cX, cY)
        #cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
        #cv2.putText(cv_image, "center", (cX - 20, cY - 20),
        #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        rect = cv2.minAreaRect(pole)
	box = cv2.boxPoints(rect)
	box = np.int0(box)
	cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 2)

        message = CameraPosition()
        message.x = cX
	if self.focalLength == None:
		_ , self.focalLength = distance.findDistance(rect,self.focalLength)
	else:
		message.y, self.focalLength = distance.findDistance(rect,self.focalLength)	

	try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            #self.camera_position.publish(message)
        except CvBridgeError as e:
            	print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt():
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
