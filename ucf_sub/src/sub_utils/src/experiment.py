from geometry_msgs.msg import Wrench
from sensor_msgs import Joy
import pandas as pd
import pickle
import time

class experiment:

	def __init__(self):
		self.teleopControl = False
		self.teleopMessage = Wrench()
		
		self.THRUST_MAX = 5
		self.duration = sys.argv[0]
		
		self.cameraPosition = None
		
		self.collectedData = []
		
		self.message = Wrench()
		
		autoCommand = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=10)
	
	    	teleopTranslate = rospy.Subscriber("/translate/joy", Joy, teleopTranslate)
	    	teleopRotate = rospy.Subscriber("/rotate/joy", Joy, teleopRotate)  	      
	
		cameraPosition = rospy.Subscriber("/camera_position", CameraPosition, setCamera)

	def teleopTranslate(self, data):
		self.teleopMessage.force.x = data.axes[1] * 20
		self.teleopMessage.force.y = data.axes[0] * 20
		self.teleopMessage.force.z = data.axes[2] * 20			      	
		self.teleopControl = data.axes[4]

	def teleopRotate(self, data):
		self.teleopMessage.torque.x = data.axes[0] * -2
		self.teleopMessage.torque.y = data.axes[1] * 2
		self.teleopMessage.torque.z = data.axes[2] * 2
					      
	def setCamera(self, data):
		self.cameraPosition = (data.x, data.y)

	def experimentRun(self):
		for thrust in np.linspace(0, self.THRUST_MAX, num=10):
			for direction in [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]:

				force = thrust * np.array(direction)
				message.force.x = force[1] * 6
				message.force.y = force[0] * 6
				message.force.z = force[2] * 6

				startTime = time.time()
				startPosition = self.cameraPosition
							
				r = rospy.Rate(30)
				
				while startTime + duration < time.time():
					r.sleep()
								
					if self.teleopControl > 0:
						thrusterCommand.publish(message)
						
					else:
						startTime = time.time()
						startPosition = self.cameraPosition
						thrusterCommand.publish(teleopMessage)
					
				endPosition = self.cameraPosition

				self.collectedData.append((message.force.x,message.force.y,message.force.z), endPosition)
					
				message.force.x = 0
				message.force.y = 0
				message.force.z = 0
				thrusterCommand.publish(message)
				
				stopTime = time.time()
				while stopTime + 5 < time.time():
					pass
					

def main(args):
	
    ex = experiment()
    rospy.init_node('experiment', anonymous=False)
	
    try:
        ex.experimentRun()
    	pickle.dump(ex.collectedData, open("experiment_data.p", "wb"))
	
    except KeyboardInterrupt():
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
