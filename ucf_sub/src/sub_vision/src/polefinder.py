import cv2
import rospy
import numpy as np
from vision_utils import ThreshAndContour, Thresholds
from sub_vision.msg import TrackObjectFeedback
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class PoleFinder:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/thresh_image", Image, queue_size=10)

	def process(self, imageLeftRect, imageRightRect, imageDisparityRect, cameraModel, stereoCameraModel):
		#imageHSV = cv2.cvtColor(imageRightRect, cv2.COLOR_BGR2HSV)
		#resized = cv2.resize(imageLeftRect, width=300)
		mask=cv2.inRange(imageRightRect, np.array([10,30,80],dtype='uint8'),np.array([80,120,225],dtype='uint8'))
		output = cv2.bitwise_and(imageRightRect, imageRightRect, mask=mask)
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
		cnts = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		contours = cnts[1]

		if len(contours) == 0:
			rospy.loginfo("Here")
			return None

		pole = []
		maxLength = 0
		#cv2.drawContours(imageRightRect, [contours], 0, (0,0,255),2)

		for c in contours:
			M = cv2.moments(c)
			if len(c)  > maxLength:
				maxLength = len(c)
				pole=np.array(c)


		x, y, w, h = cv2.boundingRect(pole)
		poleCenter = (x+(w/2),y+(h/2))
		feedback = TrackObjectFeedback()
		feedback.found = True
		feedback.center = poleCenter
		feedback.width = w
		feedback.height = h
		#rospy.loginfo(feedback)
		return feedback

