from geometry_msgs.msg import Wrench
from sensor_msgs import Joy
import pandas as pd
import pickle


class experiment:

	def __init__(self):
		self.teleopControl = False
		self.teleopMessage = Wrench()
		
		self.THRUST_MAX = 5
		self.duration = sys.argv[0]
		
		self.cameraPosition = None
		
		self.collectedData = {}
		
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

	def experiment(self):
		if self.teleopControl > 0:
		    for thrust in np.linspace(0, self.THRUST_MAX, num=10):
		    	if self.teleopControl > 0:
			    	for direction in [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]:

			    	if self.teleopControl > 0:
				        message.force = thrust * direction

				    	try:
				    		if self.teleopControl > 0:
					    		startTime = time.time()
					    		startPosition = self.cameraPosition

					    		thrusterCommand.publish(message)

					    		while startTime + duration < time.time():
					    			if self.teleopControl > 0:
					    				break
								else:
									thrusterCommand.publish(teleopMessage)
									
					    		endPosition = self.cameraPosition

					    		self.collectedData[message.force] = endPosition - startPosition
						else:
							thrusterCommand.publish(teleopMessage)
				else: 
					thrusterCommand.publish(teleopMessage)
			else:
				thrusterCommand.publish(teleopMessage)
		else:
			thrusterCommand.publish(teleopMessage)
					

def main(args):
	
    ex = experiment()
    rospy.init_node('experiment', anonymous=False)
	
    try:
        rospy.spin()
    	data = pd.from_dict(ex.collectedData)
    	pickle.dump(data, open("experiment_data.p", "wb"))
	
    except KeyboardInterrupt():
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
