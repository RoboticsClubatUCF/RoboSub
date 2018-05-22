import numpy as np
import cv2
import image_geometry

import vision_utils

from sub_vision.msg import TrackObjectFeedback

class BuoyFinder:
    def __init__(self):
        pass
        
    def process(self, imageLeftRect, imageDisparityRect, cameraModelLeft, cameraModelStereo, thresholds=vision_utils.Thresholds(upperThresh=(40,52,120), lowerThresh=(20, 30, 80))):
        
        self.cameraModel = cameraModelLeft
        self.cameraModelStereo = cameraModelStereo
        self.thresholds = thresholds
        
        imageHSV = cv2.cvtColor(imageLeftRect, cv2.COLOR_BGR2HSV)
        contours, _ = ThreshAndContour(imageHSV, thresholds)
        if len(contours) > 0:
            circles = filter(lambda x : contourCircularity(x)>0.7, contours)
        else:
            return None
            
        if len(circles) > 0:
            biggestCircle = greatestAreaContour(circles)
        else:
            return None
            
        if cv2.contourArea(biggestCircle) < 300:
            return None
            
        (x,y), radius = cv2.minEnclosingCircle(biggestCircle)
        _,_,w,h = cv2.boundingRect(biggestCircle)
        
        closest = 0
        
        if cameraModelStereo is not None:
            mask = np.zeros((imageDisparityRect.height, imageDisparityRect.width, 1), np.uint8)
            cv2.circle(mask, (x,y), radius, (255), -1)
            masked = cv2.bitwise_and(imageDisparityRect, imageDisparityRect, mask=mask)
            
            closest = masked.max()
        
        if closest > 0: #Get position based off stereo disparity (assumed(?) more accurate)
            estLocation = self.cameraModelStereo.projectPixelTo3d((x,y),closest)
        else:           #Get position based off of object size and focal length
            c = 0.2 #actual bouy size in meters
            dist = ((c*self.cameraModel.fx())/w + (c*self.cameraModel.fy())/h)/2
            projectedRay = self.cameraModel.projectPixelTo3dRay((x,y))
            estLocation = tuple(dist*x for x in projectedRay) #http://stackoverflow.com/a/1781987
        
        feedback = TrackObjectFeedback()
        feedback.fixType=feedback.position
        feedback.targetPose.Point.x = estLocation[0]
        feedback.targetPose.Point.y = estLocation[1]
        feedback.targetPose.Point.z = estLocation[2]
        
        feedback.targetPose.Quaternion.w = 1
        feedback.targetPose.Quaternion.x = 0
        feedback.targetPose.Quaternion.y = 0
        feedback.targetPose.Quaternion.z = 0
        
        return feedback
