import cv2
import numpy as np
from vision_utils import ThreshAndContour
from sub_vision.msg import TrackObjectFeedBack

class PoleFinder:
	def __init__(self):
		pass

	def process(self, imageLeftRect, imageRightRect, imageDisparityRect, cameraModel, stereoCameraModel):
		#imageHSV = cv2.cvtColor(imageLeftRect, cv2.COLOR_BGR2HSV)
		#resized = cv2.resize(imageLeftRect, width=300)
		contours, _ = ThreshAndContour(imageLeftRect, Thresholds(upper=(237,95,78.4),lower=(204,51,100)))

		if len(contours) == 0:
			return None

		pole = []

		for contour in contours:

			M = cv2.moments(c)
			if cv2.contourArea(c) > 1000:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				pole.append(c)

		pole = np.array(pole)
		x, y, w, h = boundingRect(pole)
		poleCenter = (cX,cY)

		feedback = TrackObjectFeedback()
		feedback.center = poleCenter
		feedback.width = w
		return feedback

