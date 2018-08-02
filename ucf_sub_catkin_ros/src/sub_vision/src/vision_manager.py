#! /usr/bin/env python
import rospy
import actionlib
import cv2
import image_geometry
import json
import os.path

import numpy as np

from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult
from sub_vision.cfg import ThresholdsConfig

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from wfov_camera_msgs.msg import WFOVImage
from dynamic_reconfigure.server import Server

from polefinder import PoleFinder
from gatefinder import GateFinder
from dicefinder import DiceFinder
from pathfinder import PathFinder
import vision_utils

class VisionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('track_object', TrackObjectAction, self.execute, False)
        self.server.start()

        self.lower = None
        self.upper = None
        self.targetType = None

        self.srv = Server(ThresholdsConfig, self.updateThresholds)

        self.thresholds = {int(k):v for k,v in rospy.get_param("/vision_thresholds").items()}

        self.bridge = CvBridge()

        self.leftImage = np.zeros((1,1,3), np.uint8)
        self.leftModel = image_geometry.PinholeCameraModel()
        self.newLeft = False

        self.rightImage = np.zeros((1,1,3), np.uint8)
        self.rightModel = image_geometry.PinholeCameraModel()
        self.newRight = False

        self.downImage = np.zeros((1,1,3), np.uint8)
        self.downModel = image_geometry.PinholeCameraModel()
        self.newDown = False

        self.disparityImage = np.zeros((1,1,3), np.uint8)
        self.stereoModel = image_geometry.StereoCameraModel()
        self.newDisparity = False

        self.poleFinder = PoleFinder()
        self.gateFinder = GateFinder()
        self.diceFinder = DiceFinder()
        self.pathFinder = PathFinder()
        self.response = TrackObjectResult()

        self.downSub = rospy.Subscriber('/down_camera/image_color', Image, self.downwardsCallback)
        self.downInfoSub = rospy.Subscriber('/down_camera/info', CameraInfo, self.downInfoCallback)
        
        self.stereoSub = rospy.Subscriber('/stereo/disparity', Image, self.stereoCallback)
        #self.stereoInfoSub = rospy.Subscriber('/stereo/info', CameraInfo, self.stereoInfoCallback)

        self.leftSub = rospy.Subscriber('/stereo/left/image', WFOVImage, self.leftCallback)
        #self.leftInfoSub = rospy.Subscriber('/stereo/left/image', WFOVImage, self.leftInfoCallback)
        
        self.rightSub = rospy.Subscriber('/stereo/right/image', WFOVImage, self.rightCallback)
        #self.rightInfoSub = rospy.Subscriber('/stereo/right/image', WFOVImage, self.rightInfoCallback)

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/thresh_image", Image, queue_size=10)
        

        #self.thresholds = self.loadThresholds()

    def updateThresholds(self, config, level):
        self.lower = np.array([config["lowH"], config["lowS"], config["lowL"]],dtype=np.uint8)
        self.upper = np.array([config["upH"], config["upS"], config["upL"]],dtype=np.uint8)

        if self.targetType is not None:
            self.thresholds[self.targetType] = [self.lower.tolist(), self.upper.tolist()]
            rospy.set_param("/vision_thresholds/"+str(self.targetType), [self.lower.tolist(), self.upper.tolist()])

        return config

    def setThresholds(self, lower, upper):
        self.lower = np.array(lower,dtype=np.uint8)
        self.upper = np.array(upper,dtype=np.uint8)

        self.srv.update_configuration({"lowH":lower[0], "lowS":lower[1], "lowV":lower[2], "upH":upper[0], "upS":upper[1], "upV":upper[2]})

    def execute(self, goal):
        self.targetType = goal.objectType
        self.setThresholds(*self.thresholds[self.targetType])
        self.feedback = TrackObjectFeedback()
        self.found = False

        self.running = True
        self.ok = True

        rightImageRect = np.zeros(self.rightImage.shape, self.rightImage.dtype)
        leftImageRect = np.zeros(self.leftImage.shape, self.leftImage.dtype)
        downImageRect = np.zeros(self.downImage.shape, self.downImage.dtype)

        r = rospy.Rate(60)
        while self.running and not rospy.is_shutdown():
            if self.server.is_preempt_requested() or self.server.is_new_goal_available():
                self.running = False
                continue
            
            r.sleep()
            
            if self.rightModel.width is not None and self.rightImage is not None:
                self.rightModel.rectifyImage(self.rightImage, rightImageRect)
            else:
                rospy.logwarn_throttle(1, "No right camera model")
                continue
                
            if self.leftModel.width is not None and self.leftImage is not None:
                self.leftModel.rectifyImage(self.leftImage, leftImageRect)
            else:
                rospy.logwarn_throttle(1, "No left camera model")
                continue #We need the left camera model for stuff

            if self.downModel.width is not None and self.downImage is not None:
                self.downModel.rectifyImage(self.downImage, downImageRect)
            else:
                rospy.logwarn_throttle(1, "No down camera model")
                continue #We need the down camera model for stuff
            
            if self.targetType == TrackObjectGoal.startGate:
                if self.newRight:
                    self.feedback = self.gateFinder.process(leftImageRect, rightImageRect, self.disparityImage, self.leftModel, self.stereoModel, self.upper, self.lower)
                    self.newRight = False

            elif self.targetType == TrackObjectGoal.pole:
                if self.newRight:
                    self.feedback = self.poleFinder.process(leftImageRect, rightImageRect, self.disparityImage, self.leftModel, self.stereoModel, self.upper, self.lower)
                    self.newRight = False

            elif self.targetType == TrackObjectGoal.dice:
                if self.newRight:
                    self.feedback = self.diceFinder.process(leftImageRect, rightImageRect, self.disparityImage, self.leftModel, self.stereoModel, goal.diceNum, self.upper, self.lower)
                    self.newRight = False

            elif self.targetType == TrackObjectGoal.path:
                if self.newDown:
                    self.feedback = self.pathFinder.process(downImageRect, self.downModel, self.upper, self.lower)
                    self.newDown = False

            else:
                self.ok = False
                break

            if self.feedback.found:
                self.server.publish_feedback(self.feedback)
                self.feedback.found = False
                self.response.found=True
            if not goal.servoing:
                self.running = False

        self.response.stoppedOk=self.ok
        self.server.set_succeeded(self.response)

#TODO: Fix color thresholds

    def downwardsCallback(self, msg):
        try:
            self.downImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.newDown = True
        except CvBridgeError as e:
            print(e)

        #self.downModel.rectifyImage(self.downImage, self.downImage)

    def stereoCallback(self, msg):
        try:
            self.disparityImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.newDisparity = True
        except CvBridgeError as e:
            print(e)

    def leftCallback(self, msg):
        try:
            self.leftImage = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
            self.leftModel.fromCameraInfo(msg.info)
            self.newLeft = True
        except CvBridgeError as e:
            print(e)

    def rightCallback(self, msg):
        try:
            self.rightImage = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
            self.rightModel.fromCameraInfo(msg.info)
            self.newRight = True
        except CvBridgeError as e:
            print(e)


    def downInfoCallback(self, msg):
        self.downModel.fromCameraInfo(msg)

    def rightInfoCallback(self, msg):
        self.rightModel.fromCameraInfo(msg.info)

    def leftInfoCallback(self, msg):
        self.leftModel.fromCameraInfo(msg.info)

    def loadThresholds(self):
        #with open(os.path.dirname(__file__) + '/../thresholds.json') as data_file:
            #json_data = json.load(data_file)
        data = {}
        #for entry in json_data:
            # high = entry['high']
            #low = entry['low']
            #data[entry['color']] = vision_utils.Thresholds(upperThresh=(high['hue'],high['sat'],high['val']), lowerThresh=(low['hue'],low['sat'],low['val']))
        return data

if __name__ == '__main__':
    rospy.init_node('vision_server')
    server = VisionServer()
    rospy.spin()
