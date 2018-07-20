#! /usr/bin/env python
import rospy
import actionlib
import cv2
import image_geometry
import json
import os.path

import numpy as np

from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from wfov_camera_msgs.msg import WFOVImage

from polefinder import PoleFinder
from gatefinder import GateFinder
from dicefinder import DiceFinder
import navbarfinder, bouyfinder, vision_utils

class VisionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('track_object', TrackObjectAction, self.execute, False)
        self.server.start()

        self.bridge = CvBridge()

        self.leftImage = np.zeros((1,1,3), np.uint8)
        self.leftModel = image_geometry.PinholeCameraModel()

        self.rightImage = np.zeros((1,1,3), np.uint8)
        self.rightModel = image_geometry.PinholeCameraModel()

        self.downImage = np.zeros((1,1,3), np.uint8)
        self.downModel = image_geometry.PinholeCameraModel()

        self.disparityImage = np.zeros((1,1,3), np.uint8)
        self.stereoModel = image_geometry.StereoCameraModel()

        self.poleFinder = PoleFinder()
        self.gatefinder = GateFinder()
        self.dicefinder = DiceFinder()
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

    def execute(self, goal):
        self.targetType = goal.objectType
        self.feedback = TrackObjectFeedback()

        self.running = True
        self.ok = True

        r = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            rightImageRect = np.zeros(self.rightImage.shape, self.rightImage.dtype)
            leftImageRect = np.zeros(self.leftImage.shape, self.leftImage.dtype)

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

            if self.server.is_preempt_requested() or self.server.is_new_goal_available():
                    self.running = False
                    continue

            elif self.targetType == TrackObjectGoal.startGate:
                self.feedback = self.gatefinder.process(leftImageRect, rightImageRect, self.disparityImage, self.leftModel, self.stereoModel)
                if self.feedback.found:
                    self.server.publish_feedback(self.feedback)
                    self.feedback.found = False
                    self.response.found=True
                if not goal.servoing:
                    self.running = False

            elif self.targetType == TrackObjectGoal.pole:
                self.feedback = self.poleFinder.process(leftImageRect, rightImageRect, self.disparityImage, self.leftModel, self.stereoModel)
                if self.feedback.found:
                    self.server.publish_feedback(self.feedback)
                    self.feedback.found = False
                    self.response.found=True
                if not goal.servoing:
                    self.running = False

            elif self.targetType == TrackObjectGoal.dice:
                self.feedback = self.diceFinder.process(leftImageRect, rightImageRect, self.disparityImage, self.leftModel, self.stereoModel,goal.diceNum)
                if self.feedback.found:
                    self.server.publish_feedback(self.feedback)
                    self.feedback.found = False
                    self.response.found=True
                if not goal.servoing:
                    self.running = False

            r.sleep()

        self.response.stoppedOk=self.ok
        self.server.set_succeeded(self.response)

#TODO: Fix color thresholds

    def downwardsCallback(self, msg):
        try:
            self.downImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.downModel is None:
            print("No camera model for downwards camera")
            return

        #self.downModel.rectifyImage(self.downImage, self.downImage)

    def stereoCallback(self, msg):
        try:
            self.disparityImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.stereoModel is None:
            print("No camera model for stereo camera")
            return

    def leftCallback(self, msg):
        try:
            self.leftImage = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
            self.leftModel.fromCameraInfo(msg.info)
        except CvBridgeError as e:
            print(e)

    def rightCallback(self, msg):
        try:
            self.rightImage = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
            self.rightModel.fromCameraInfo(msg.info)
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
