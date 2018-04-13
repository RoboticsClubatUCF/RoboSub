#! /usr/bin/env python2
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

colors = []

class threshold_finder:

    def __init__(self):
	self.frame = None
	self.colors = []
	self.hasNewFrame = False
        self.thresholds = np.array([0,0,0,0,0,0], dtype= np.float32)
        self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/contoured_image", Image, self.callback)
        self.image_thresholds = rospy.Publisher("/threshold_values",numpy_msg(Floats), queue_size=10)

    def pick_colors(self, event, x, y, flags, frame):
        if event == cv2.EVENT_LBUTTONUP:
            self.colors.append(self.frame[y,x].tolist())
	    minb = min(c[0] for c in self.colors)
            ming = min(c[1] for c in self.colors)
            minr = min(c[2] for c in self.colors)
	    maxb = max(c[0] for c in self.colors)
            maxg = max(c[1] for c in self.colors)
            maxr = max(c[2] for c in self.colors)
            print (minr, ming, minb, maxr, maxg, maxb)

            lb = [minb,ming,minr]
            ub = [maxb,maxg,maxr]
            print (lb, ub)

	
    def callback(self,data):
	try:
       		self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
                print(e)
	if colors:
        	cv2.putText(frame, str(colors[-1]), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
	self.hasNewFrame=True

	try:
        	minb = min(c[0] for c in self.colors)
         	for x in range(x):
            		if x < 3:
                		thresholds[x] = max(c[x] for c in self.colors)
            		else:
                		thresholds[x] = min(c[x%3] for c in self.colors)
	except:
		pass

        try:
        	self.image_thresholds.publish(thresholds)
	except:
		pass

def main(args):
        tf = threshold_finder()
        rospy.init_node('threshold_finder', anonymous=False)
        cv2.namedWindow('Sub Camera')
	cv2.setMouseCallback('Sub Camera', tf.pick_colors, tf.frame)
        try:
            while not rospy.is_shutdown():
		if tf.frame is not None and tf.hasNewFrame:
			cv2.imshow('Sub Camera',tf.frame)
			key=cv2.waitKey(1)
			if key ==27:
				sys.exit()	
			tf.hasNewFrame = False
        except KeyboardInterrupt():
            print("Shutting down")

        cv2.destroyAllWindows()


if __name__ == "__main__":
        main(sys.argv)
