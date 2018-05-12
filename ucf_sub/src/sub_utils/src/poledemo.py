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

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher(
            "/contoured_image", Image, queue_size=10)
        self.lower = np.array([9, 27, 100], dtype="uint8")
        self.upper = np.array([39, 83, 247], dtype="uint8")
	self.bridge = CvBridge()
        self.focalLength = None
        self.tracker = cv2.TrackerKCF_create()
	self.ok = None
	self.image_sub = rospy.Subscriber(
            "/stereo/right/image_color", Image, self.callback)
        
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

	x,y,w,h = cv2.boundingRect(pole)
        #rect = cv2.minAreaRect(pole)
	#box = cv2.boxPoints(rect)
	#bbox = cv2.selectROI(cv_image,box)
	#box = np.int0(box)

	if self.ok == None:
		self.ok = self.tracker.init(cv_image, (x,y,w,h))
	else:
		self.ok, box = self.tracker.update(cv_image)

	try:
		p1 = (int(box[0]), int(box[1]))
		p2 = (int(box[0] + box[2]), int(box[1] + box[3]))
		cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)
	except:
		pass

	try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
