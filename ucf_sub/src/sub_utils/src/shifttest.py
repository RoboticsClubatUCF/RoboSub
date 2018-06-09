import rospy
import numpy as np
import pandas as pd
from sensor_msgs.msg import CameraInfo
import sys

class recordMovement:
	def __init__(self):
		self.cameraInfo = None
		self.positionSubscriber = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, self.findInfo)

	def findInfo(self, data):
		print('here')
		if self.cameraInfo is None:
			print(data)
			self.cameraINfo = data

def main(args):
	rm = recordMovement()
	rospy.init_node('recordMovement', anonymous=False)
	rospy.spin()


if __name__ == "__main__":
        main(sys.argv)
