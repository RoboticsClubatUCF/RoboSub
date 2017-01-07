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
            
            gateLocation = None
            gateAxis = None
            gateAngle = None
            if stereoCameraModel is not None: #If we get this, we have(?) stereo camera data, so we can do a full 3d pose
                mask1 = np.zeros((imageDisparityRect.height, imageDisparityRect.width, 1), np.uint8)
                roi1Center = (rect1[0][0] + rect1[1][0]*0.4*cos(rect1[2]), rect1[0][0] + rect1[1][1]*0.4*sin(rect1[2]))
                cv2.circle(mask1, roi1Center, min(rect1[1])*0.5, (255), -1)
                
                mask2 = np.zeros((imageDisparityRect.height, imageDisparityRect.width, 1), np.uint8)
                roi2Center = (rect1[0][0] - rect1[1][0]*0.4*cos(rect1[2]), rect1[0][0] - rect1[1][1]*0.4*sin(rect1[2]))
                cv2.circle(mask2, roi2Center, min(rect1[1])*0.5, (255), -1)
                
                mask3 = np.zeros((imageDisparityRect.height, imageDisparityRect.width, 1), np.uint8)
                roi3Center = (rect2[0][0] + rect2[1][0]*0.4*cos(rect2[2]), rect2[0][0] + rect2[1][1]*0.4*sin(rect2[2]))
                cv2.circle(mask3, roi3Center, min(rect2[1])*0.5, (255), -1)
                
                mask4 = np.zeros((imageDisparityRect.height, imageDisparityRect.width, 1), np.uint8) 
                roi4Center = (rect2[0][0] - rect2[1][0]*0.4*cos(rect2[2]), rect2[0][0] - rect2[1][1]*0.4*sin(rect2[2]))
                cv2.circle(mask4, roi4Center, min(rect2[1])*0.5, (255), -1)
                
                disp[1] = cv2.bitwise_and(imageDisparityRect, imageDisparityRect, mask=mask1).max() #determine disparity on all the gate corners
                disp[2] = cv2.bitwise_and(imageDisparityRect, imageDisparityRect, mask=mask2).max()
                disp[3] = cv2.bitwise_and(imageDisparityRect, imageDisparityRect, mask=mask3).max()
                disp[4] = cv2.bitwise_and(imageDisparityRect, imageDisparityRect, mask=mask4).max()
                
                if disp.count(0) < 1: #we can still get pose with 3 disparity readings
                    corner = []
                    corner[1] = stereoCameraModel.projectPixelTo3d(roi1Center,disp[1]) #Find all 4 gate corners in 3d
                    corner[2] = stereoCameraModel.projectPixelTo3d(roi2Center,disp[2])
                    corner[3] = stereoCameraModel.projectPixelTo3d(roi3Center,disp[3])
                    corner[4] = stereoCameraModel.projectPixelTo3d(roi4Center,disp[4])
                    
                    normals = self.normalsFromAllCorners(corner, disp) #generates normals from all the combinations of corners, *should* be the right way round
                    
                    gatenormal = np.sum(normals, axis=0)
                    gatenormal = np.divide(gatenormal, len(normals))
                    
                    cos_theta = np.dot(gatenormal, np.array([0, 0, 1]))
                    gateAngle = acos(cos_theta)
                    
                    gateAxis = np.cross(gatenormal, np.array([0, 0, 1]))
                    
                    if np.abs(gateAxis).max() > 0:
                        gateAxis = np.divide(gateAxis, np.abs(gateAxis).max()) #normalize
                    
                    validCorners = []
                    for idx, val in enumerate(corners):
                        if disp[idx] > 0:
                            validCorners.append(val)
                    gateLocation = np.divide(np.sum(validCorners, axis=0), len(validCorners))
                    
                    
            if gateLocation is None: #no stereo, center position only
                gatewidth = abs(rect1[0][0] - rect2[0][0])
                gateheight = abs(rect1[0][1] - rect2[0][1])
                
                gatesize = sqrt(gatewidth*gatewidth + gateheight*gateheight)
                
                c = 3.05 #actual gate size constant
                gateDist = c * (self.cameraModel.fx()*(gatesize/gatewidth)+self.cameraModel.fy()*(gatesize/gateheight))/gatesize
                #I *think* this accounts for rotational wierdness in the focal lengths
                
                gateCenter = ((rect1[0][0] + rect2[0][0])/2, (rect1[0][1] + rect2[0][1])/2)
                projectedRay = cameraModel.projectPixelTo3dRay(gateCenter)
                gateLocation = tuple(gateDist*x for x in projectedRay)
            
            if gateLocation is not None:    
                feedback.targetPose.Point.x = gateLocation[0]
                feedback.targetPose.Point.y = gateLocation[1]
                feedback.targetPose.Point.z = gateLocation[2]
                if gateAxis is None:
                    feedback.fixType=feedback.position
                    feedback.targetPose.Quaternion.w = 1
                    feedback.targetPose.Quaternion.x = 0
                    feedback.targetPose.Quaternion.y = 0
                    feedback.targetPose.Quaternion.z = 0
                else:    
                    feedback.fixType=feedback.pose
                    feedback.targetPose.Quaternion.w = cos(gateAngle/2)
                    feedback.targetPose.Quaternion.x = gateAxis[0] * sin(gateAngle/2)
                    feedback.targetPose.Quaternion.y = gateAxis[1] * sin(gateAngle/2)
                    feedback.targetPose.Quaternion.z = gateAxis[2] * sin(gateAngle/2)
                return feedback
            else:
                return None
        else:
            return None
            

