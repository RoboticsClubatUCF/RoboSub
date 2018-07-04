import cv2
import rospy, time
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
		
		imageHLS = cv2.cvtColor(imageRightRect, cv2.COLOR_BGR2HLS)
		#resized = cv2.resize(imageLeftRect, width=300)
		
		mask=cv2.inRange(imageHLS, np.array([5, 102, 180],dtype='uint8'),np.array([25, 153, 255],dtype='uint8')) #HSV thresholds
		
		kernel = np.ones((5,5),np.uint8)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		#mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
		output = cv2.bitwise_and(imageRightRect, imageRightRect, mask=mask)

		_, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		if len(cnts) == 0:
			rospy.logwarn("Here")
			return None

		pole = []
		bestFilled = 0
		bestArea = 0
		
		for c in cnts:
			
			area = cv2.contourArea(c)
			if area < 500:
				continue
			
			#c = cv2.approxPolyDP(c, 0.001*cv2.arcLength(c, True), False)
			#area = cv2.contourArea(c)
			rect = cv2.fillPoly(np.zeros(mask.shape), [c], (255))
			rect = cv2.bitwise_and(rect, rect, mask=mask)
			filled = cv2.countNonZero(rect)
			if area/filled > bestFilled:
				bestFilled = area/filled
				bestArea = area
				pole = c
			elif area/filled >= (0.75 * bestFilled) and area > bestArea:
				bestFilled = area/filled
				bestArea = area
				pole = c

		
		output = cv2.fillPoly(output, [np.int0(cv2.boxPoints(cv2.minAreaRect(pole)))], (255,255,255))
		output = cv2.bitwise_and(output, output, mask=mask)

		output = cv2.drawContours(output, [c for c in cnts if cv2.contourArea(c) > 500], -1, (0,255,0),1)
		output = cv2.drawContours(output, [cv2.approxPolyDP(c, 0.001*cv2.arcLength(c, True), False) for c in cnts if cv2.contourArea(c) > 500], -1, (255,0,0),1)

		rects = [cv2.boundingRect(c) for c in cnts if cv2.contourArea(c) > 100]
		for r in rects:
			output = cv2.rectangle(output, (r[0], r[1]), (r[0]+r[2], r[1]+r[3]), (255,0,0))

		x, y, w, h = cv2.boundingRect(pole)
		output = cv2.rectangle(output, (x,y), (x+w,y+h), (0,255,0))
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
		poleCenter = (x+(w/2),y+(h/2))
		feedback = TrackObjectFeedback()
		feedback.found = True
		feedback.center = poleCenter
		feedback.width = w
		feedback.height = h
		#rospy.loginfo(feedback)
		return feedback

