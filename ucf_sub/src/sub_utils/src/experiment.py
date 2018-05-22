from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
import pandas as pd
import pickle
import time
import sys
import rospy
from sub_utils.msg import CameraPosition
import numpy as np

class experiment:

	def __init__(self):
		self.teleopControl = True
		self.teleopMessage = Wrench()
		
		self.THRUST_MAX = 5
		self.duration = 600000.0
		
		self.cameraPosition = None
		
		self.collectedData = []
		
		self.message = Wrench()
		
		self.autoCommand = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=10)
	
	    	teleopTranslate = rospy.Subscriber("/translate/joy", Joy, self.teleopTranslate)
	    	teleopRotate = rospy.Subscriber("/rotate/joy", Joy, self.teleopRotate)
		cameraPosition = rospy.Subscriber("/camera_position", CameraPosition, self.setCamera)

	def teleopTranslate(self, data):
		self.teleopMessage.force.x = data.axes[1] * 2
		self.teleopMessage.force.y = data.axes[0] * 2
		self.teleopMessage.force.z = data.axes[2] * 2
		self.teleopControl = data.axes[4]
		if self.teleopcontrol < 0:
			self.teleopcontrol = False

	def teleopRotate(self, data):
		self.teleopMessage.torque.x = data.axes[0] * -2
		self.teleopMessage.torque.y = data.axes[1] * 2
		self.teleopMessage.torque.z = data.axes[2] * 2

	def setCamera(self, data):
		self.cameraPosition = (data.x, data.y)

	def experimentRun(self):
		r = rospy.Rate(30)
		for thrust in np.linspace(0, self.THRUST_MAX, num=5):
			for direction in [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]:
				duration = 100000.0
				force = thrust * np.array(direction)
				print(force)
				self.message.force.x = force[1]
				self.message.force.y = force[0]
				self.message.force.z = force[2]
                                startPosition = self.cameraPosition
				startTime = time.time() + 1
				while time.time() < startTime :
					r.sleep()
					if self.teleopControl:
						self.autoCommand.publish(self.message)
					else:
						startTime = time.time() + 1
						startPosition = self.cameraPosition
						self.autoCommand.publish(self.teleopMessage)	
				endPosition = self.cameraPosition
				self.collectedData.append(((self.message.force.x,self.message.force.y,self.message.force.z), endPosition))

				self.message.force.x = 0
				self.message.force.y = 0
				self.message.force.z = 0
				self.autoCommand.publish(self.message)

				stopTime = time.time() + 1
				while stopTime < time.time():
					pass

def main(args):

    ex = experiment()
    rospy.init_node('experiment', anonymous=False)
    ex.experimentRun()
    pickle.dump(ex.collectedData, open("experiment_data.p", "wb"))

if __name__ == '__main__':
    main(sys.argv)
