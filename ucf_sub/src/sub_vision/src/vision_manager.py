#! /usr/bin/env python
import rospy
import actionlib
import cv2
import image_geometry
import json
import os.path

from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from polefinder import PoleFinder
import  gatefinder, navbarfinder, bouyfinder, vision_utils

class VisionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('track_object', TrackObjectAction, self.execute, False)
        self.server.start()

        self.bridge = CvBridge()

        self.downSub = rospy.Subscriber('/down_camera/image_raw', Image, self.downwardsCallback)
        self.downInfoSub = rospy.Subscriber('/down_camera/info', CameraInfo, self.downInfoCallback)
        self.downImage = None
        self.downModel = None
        self.stereoSub = rospy.Subscriber('/stereo/disparity', Image, self.stereoCallback)
        #self.stereoInfoSub = rospy.Subscriber('/stereo/info', CameraInfo, self.stereoInfoCallback)
        self.disparityImage = None
        self.stereoModel = None

        self.leftSub = rospy.Subscriber('/stero/left/image_raw', Image, self.leftCallback)
        self.leftInfoSub = rospy.Subscriber('/stereo/left/camera_info', CameraInfo, self.leftInfoCallback)
        self.leftImage = None
        self.leftModel = None
        self.leftMsg = None
        self.rightSub = rospy.Subscriber('/stereo/right/image_raw', Image, self.rightCallback)
        self.rightInfoSub = rospy.Subscriber('/stereo/right/camera_info', CameraInfo, self.rightInfoCallback)
        self.rightImage = None
        self.rightModel = None
        self.rightMsg = None

        #self.thresholds = self.loadThresholds()

        self.poleFinder = PoleFinder()
        self.gatefinder = gatefinder.GateFinder()
        self.response = TrackObjectResult()

    def execute(self, goal):
        self.targetType = goal.objectType
	self.feedback = TrackObjectFeedback()

        self.running = True
        self.ok = True

        if self.leftMsg is not None and self.rightMsg is not None:
            self.stereoModel = image_geometry.StereoCameraModel()
            self.stereoModel.fromCameraInfo(self.leftMsg, self.rightMsg)

        while self.running: 
       		if self.server.is_preempt_requested() or self.server.is_new_goal_available():
			self.running = False
                	continue

            	elif self.targetType == TrackObjectGoal.startGate:
 	        	self.feedback = self.gatefinder.process(self.leftImage, self.rightImage, self.disparityImage, self.leftModel, self.stereoModel)
                	if self.feedback.found:
                    		self.server.publish_feedback(self.feedback)
                    		self.feedback.found = False
                    		self.response.found=True
                	if not goal.servoing:
                    		self.running = False

            	elif self.targetType == TrackObjectGoal.pole:
			self.feedback = self.poleFinder.process(self.leftImage,self.rightImage,self.disparityImage,self.leftModel,self.stereoModel)
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
            self.downImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.downModel is None:
            print("No camera model for downwards camera")
            return

        self.downModel.rectifyImage(self.downImage, self.downImage)

    def stereoCallback(self, msg):
        try:
            self.disparityImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.stereoModel is None:
            print("No camera model for stereo camera")
            return

        self.stereoModel.rectifyImage(self.disparityImage, self.disparityImage)

    def leftCallback(self, msg):
        try:
            self.leftImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.leftModel is None:
            print("No camera model for left camera")
            return

        self.leftModel.rectifyImage(self.leftImage, self.leftImage)

    def rightCallback(self, msg):
        try:
            self.rightImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.rightModel is None:
            print("No camera model for right camera")
            return

        #self.rightModel.rectifyImage(self.rightImage, self.rightImage)

    def downInfoCallback(self, msg):
        self.downModel = image_geometry.PinholeCameraModel()
        self.downModel.fromCameraInfo(msg)

    def rightInfoCallback(self, msg):
        self.rightMsg = msg
        self.rightModel = image_geometry.PinholeCameraModel()
        self.rightModel.fromCameraInfo(msg)

    def leftInfoCallback(self, msg):
        self.leftMsg = msg
        self.leftModel = image_geometry.PinholeCameraModel()
        self.leftModel.fromCameraInfo(msg)

    def loadThresholds(self):
        with open(os.path.dirname(__file__) + '/../thresholds.json') as data_file:
            json_data = json.load(data_file)
        data = {}
        for entry in json_data:
            high = entry['high']
            low = entry['low']
            data[entry['color']] = vision_utils.Thresholds(upperThresh=(high['hue'],high['sat'],high['val']), lowerThresh=(low['hue'],low['sat'],low['val']))
        return data

if __name__ == '__main__':
    rospy.init_node('vision_server')
    server = VisionServer()
    rospy.spin()
