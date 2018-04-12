#! /usr/bin/env python2
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Image
from rospy_tutorials.msg import Floats
from rospy.numpy_nmsg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

colors = []

class threshold_finder:

    def __init__(self):
        self.thresholds = np.array([0,0,0,0,0,0], dtype= "numpy.float32")
        self.image_sub = rospy.Subscriber("/contoured_image", Image, self.callback)
        self.image_thresholds = rospy.Publisher("/threshold_values",numpy_msg(Floats), queue_size=10) 

    def on_mouse_click (event, x, y, flags, frame):
        if event == cv2.EVENT_LBUTTONUP:
            colors.append(frame[y,x].tolist())

    def callback():
        if colors:
            cv2.putText(frame, str(colors[-1]), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
            cv2.imshow('frame', frame)
            cv2.setMouseCallback('frame', on_mouse_click, frame)

            key = cv2.waitKey(1)
            if key == 27:
                break

        minb = min(c[0] for c in colors)
        ming = min(c[1] for c in colors)
        minr = min(c[2] for c in colors)
        maxb = max(c[0] for c in colors)
        maxg = max(c[1] for c in colors)
        maxr = max(c[2] for c in colors)
        print (minr, ming, minb, maxr, maxg, maxb)

        for x in range(x):
            if x < 3:
                thresholds[x] = max(c[x] for c in colors)
            else:
                thresholds[x] = min(c[x%3] for c in colors)

        try:
            self.image_thresholds.publish(thresholds)

        lb = [minb,ming,minr]
        ub = [maxb,maxg,maxr]
        print (lb, ub)

    def main(args):
        tf = threshold_finder
        rospy.init_node('threshold_finder', anonymous=False)
        try:
            rospy.spin()
        except KeyboardInterrupt():
            print("Shutting down")

         cv2.destroyAllWindows()


    if __name__ == "__main__":
        main()