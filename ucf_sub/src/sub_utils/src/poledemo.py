#!/usr/bin/env python

import roslib
roslib.load_manifest('package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2", Image)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		imageHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		resized = imutils.resize(imageHSV, width=300)
		contours, _ = ThreshAndContour(resized, Thresholds(upper=(237,95,78.4),lower=(204,51,100)))


		if len(contours) == 0:
		return None

		pole = []

		for contour in contours:

			M = cv2.moments(c)
			if cv2.contourArea(c) > 1000:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				pole.append(c)
				poleCenter = (cX,cY)

		cv2.drawContours(cv_image, [pole], 0, (0,255,0), 3)
		cv2.circle(cv_image, (cx,cy), 10, (0,0,255), -1)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)

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