import numpy as np
import cv2
import image_geometry

import vision_utils

from sub_vision.msg import TrackObjectFeedback


class NavbarFinder:
    def __init__(self):
        pass
        
    def process(self, imageDown, downCameraModel):
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
        
        if len(rects) > 0:
            rect = greatestAreaContour(rects)
            
            if(rect[1][0] < rect[1][1]): #Fix wonky angles from opencv (I think)
                rect[2] = (rect[2] + 180) * 180/3.141
            else:
                rect[2] = (rect[2] + 90) * 180/3.141
                
            rectWidth = min(rect[1][0], rect[1][1])
                
            c = 0.15 #actual marker width constant
            dist = c * (self.cameraModel.fx()*cos(rect[2])+self.cameraModel.fy()*sin(rect[2]))/rectWidth
            
            projectedRay = downCameraModel.projectPixelTo3dRay((rect[0][0],rect[0][1]))
            estLocation = tuple(dist*x for x in projectedRay) #http://stackoverflow.com/a/1781987
                
            feedback.fixType=feedback.pose
            feedback.targetPose.Point.x = estLocation[0]
            feedback.targetPose.Point.y = estLocation[1]
            feedback.targetPose.Point.z = estLocation[2]
            
            feedback.targetPose.Quaternion.w = cos(rect[2]/2)
            feedback.targetPose.Quaternion.x = 0
            feedback.targetPose.Quaternion.y = 0
            feedback.targetPose.Quaternion.z = sin(rect[2]/2)
            
            return feedback
        else:
            return None
            
            
