#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("contoured_image", Image, queue_size=10)
		self.lower = np.array([44,54,88], dtype = "uint8")
    		self.upper = np.array([67,110,251], dtype = "uint8")
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.callback)
		self.image_thresholds = rospy.Subscriber("/threshold_values", numpy_msg(Floats), self.getThresholds)

	def getThresholds(self, data):
		self.upper = data[0:3]
		self.lower = data[3:6]
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#imageHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		#resized = imutils.resize(imageHSV, width=300)
		
		mask = cv2.inRange(cv_image, upper, lower)
    		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        		cv2.CHAIN_APPROX_SIMPLE)

    		output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    		cnts = cnts[1] 
    		print(cnts)
    		cX = 0
    		cY = 0
    		maxLength = 0
    		pole = []

		for c in cnts:
			M = cv2.moments(c)
			if len(c) > maxLength:
			    try:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			    except:
				x=1
			    maxLength = len(c)
			    pole = [c]

        	cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
		cv2.putText(cv_image, "center", (cX - 20, cY - 20),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		cv2.drawContours(cv_image, pole, -1, (0, 255, 0), 2)

		

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt():
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
