import numpy as np
import cv2

import math

import vision_utils

from sub_vision.msg import TrackObjectFeedback

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

class GateFinder:
    def __init__(self):
        pass

    def normalsFromAllCorners(corners, disparities):
        valid = []
        for idx, val in enumerate(disparites):
            if val > 0:
                valid.append(idx)

        combos = cartesian(valid, valid, valid)

        normals = []
        for combo in combos: #Find all possible cross products of the available points
            if combo[0] != combo[1] and combo[1] != combo[2] and combo[0] != combo[2]:
                new = np.cross(corners[combo[1]] - corners[combo[0]], corners[combo[2]] - corners[combo[0]])
                if new.max() > 0:
                    new = np.divide(new, new.max()) #normalize

                if np.dot(new, np.array([-1,0,0])) < 0:
                    normals.append(-new)
                else:
                    normals.append(new)

        return normals

    def process(self, imageLeftRect, imageRightRect, imageDisparityRect, cameraModel, stereoCameraModel):

        imageHSV = cv2.cvtColor(imageLeftRect, cv2.COLOR_BGR2HSV)
        contours, _ = ThreshAndContour(imageHSV, Thresholds(upper=(40,52,120), lower=(20, 30, 80)))

        if len(contours) == 0:
            return None

        rects = []
        for contour in contours: #adapted from https://github.com/opencv/opencv/blob/master/samples/python/squares.py
            epsilon = cv2.arcLength(contour, True)*0.05
            contour = cv2.approxPolyDP(contour, epsilon, True)
            if len(contour) == 4 and cv2.isContourConvex(contour):
                contour = contour.reshape(-1, 2)
                max_cos = np.max([angle_cos( contour[i], contour[(i+1) % 4], contour[(i+2) % 4] ) for i in xrange(4)])
                if max_cos < 0.1:
                    rects.append(contour)

        feedback = TrackObjectFeedback()

	if len(rects) > 1:
	        rects = greatestNAreaContours(rects, 2)
        	rect1 = cv2.minAreaRect(rects[0])
            	rect2 = cv2.minAreaRect(rects[1])

            	if(rect1[1][0] < rect1[1][1]): #Fix wonky angles from opencv (I think)
                	rect1[2] = (rect1[2] + 180) * 180/3.141
            	else:
                	rect1[2] = (rect1[2] + 90) * 180/3.141

            	if(rect2[1][0] < rect2[1][1]):
                	rect2[2] = (rect2[2] + 180) * 180/3.141
        	else:
                	rect2[2] = (rect2[2] + 90) * 180/3.141

		gateCenter = (int((rect1[0][0] + rect2[0][0])/2), int((rect1[0][1] + rect2[0][1])/2))
		feedback.center = gateCenter
        feedback.found = True
		return feedback


        else:
            return None


