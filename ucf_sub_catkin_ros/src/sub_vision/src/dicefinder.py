import numpy as np
import cv2 as cv
import rospy, time
from sub_vision.msg import TrackObjectFeedback
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DiceFinder:
    def __init__(self):
        self.bridge = CvBridge()

    def process(self, imageLeftRect, imageRightRect, imageDisparityRect, cameraModel, stereoCameraModel, desiredDice, upper, lower):
        dice = self.findDice(imageRightRect)
        diceOrder = self.classifyDice(imageRightRect,dice)

        neededDice = dice[diceOrder.index(desiredDice-2)]

        feedback = TrackObjectFeedback()

        feedback.found = True
        feedback.center = neededDice[0]
        feedback.width = neededDice[1]
        feedback.height = neededDice[2]

        return feedback

    def angle_cos(self, p0, p1, p2):
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

    def findDice(self,img):
        img = cv.GaussianBlur(img, (5, 5), 0)
        
        squares = []
        
        _retval, bin = cv.threshold(img, 156, 255, cv.THRESH_BINARY)
        bin, contours, _hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            cnt_len = cv.arcLength(cnt, True)
            cnt = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
            
            if len(cnt) == 4 and cv.contourArea(cnt) > 1000 and cv.isContourConvex(cnt):
                cnt = cnt.reshape(-1, 2)
                max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4]) for i in range(4)])
                
                if max_cos < 0.1:
                    rect = cv.minAreaRect(cnt)
                    
                    if rect[1][0] < img.shape[0] * 0.9:
                        if rect[1][1] < img.shape[1] * 0.9:
                            #coordinates = (int(rect[0][0]-rect[1][0]/2),int(rect[0][0]+rect[1][0]/2),int(rect[0][1]-rect[1][1]/2), int(rect[0][1]+rect[1][1]/2))
                            squares.append(rect)
        return squares

    def classifyDice(self,img,dice):
        diceOrder = []
        
        for s in dice:
            
            dots = 0
            
            width = int(s[1][0])
            height = int(s[1][1])
            
            farX = int(s[0][1]-width/2)
            farY = int(s[0][0]-width/2)
            
            #dice = img[farX:farX+width, farY:farY+height]

            for i in range(3):
                prevY = int(farY+(height/3)*i)
                nextY = int(farY+(height/3)*(i+1))
                
                for j in range(3):

                    prevX = int(farX+(width/3)*j)
                    nextX = int(farX+(width/3)*(j+1))

                     #region = img[prevX:nextX, prevY:nextY]

                    fourthWidth = int((nextX-prevX)/4)
                    fourthHeight = int((nextY-prevY)/4)
                    
                    centerSquare = img[prevX+fourthWidth:nextX-fourthWidth, prevY+fourthHeight:nextY-fourthHeight]

                    sum = 0

                    for x in np.nditer(centerSquare):
                        sum = sum + x

                    mean = sum/(centerSquare.shape[0]*centerSquare.shape[1])

                    if mean < 127:
                        dots = dots + 1

                    if i == 2 and j == 2:
                        diceOrder.append(dots)          
                        
        return diceOrder