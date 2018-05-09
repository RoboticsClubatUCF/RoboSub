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
		self.image_sub = rospy.Subscriber("/stereo/right/image_rect_color", Image, self.callback)
		self.image_thresholds = rospy.Subscriber("/threshold_values", numpy_msg(Floats), self.getThresholds)

	def getThresholds(self, data):
		self.upper = data[0:3]
		self.lower = data[3:6]
		
	def callback(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		imageHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    	mask, contours = ThreshAndContour(imageHSV, (40,52,120), (20, 30, 80))
    	output = cv2.bitwise_and(img, img, mask=mask)

    	if len(contours) == 0:
        	return None
            
	    rects = []
	    for contour in contours[1]: #adapted from https://github.com/opencv/opencv/blob/master/samples/python/squares.py
	        epsilon = cv2.arcLength(contour, True)*0.05
	        contour = cv2.approxPolyDP(contour, epsilon, True)
	        if len(contour) == 4 and cv2.isContourConvex(contour):
	            contour = contour.reshape(-1, 2)
	            max_cos = np.max([angle_cos( contour[i], contour[(i+1) % 4], contour[(i+2) % 4] ) for i in range(4)])
	            if max_cos < 0.1:
	                rects.append(contour)
	        
	        if len(rects) > 1:
	            rects = greatestNAreaContours(rects, 2)
	            rect1 = cv2.minAreaRect(rects[0])
	            rect2 = cv2.minAreaRect(rects[1])
	            box = cv2.boxPoints(rect1)
	            box = np.int0(box)
	            cv2.drawContours(img,[box],5,(0,0,255),2)
	            box = cv2.boxPoints(rect2)
	            box = np.int0(box)
	            cv2.drawContours(img,[box],5,(0,0,255),2)

	            if(rect1[1][0] < rect1[1][1]): #Fix wonky angles from opencv (I think)
	                rect1 = (rect1[0], rect1[1], (rect1[2] + 180) * 180/3.141)
	            else:
	                rect1 = (rect1[0], rect1[1], (rect1[2] + 90) * 180/3.141)
	                
	            if(rect2[1][0] < rect2[1][1]):
	                rect2 = (rect2[0], rect2[1], (rect2[2] + 180) * 180/3.141)
	            else:
	                rect2 = (rect2[0], rect2[1], (rect2[2] + 90) * 180/3.141)
	            
	            gateLocation = None
	            gateAxis = None
	            gateAngle = None                     
	            gateCenter = (int((rect1[0][0] + rect2[0][0])/2), int((rect1[0][1] + rect2[0][1])/2))
		

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
