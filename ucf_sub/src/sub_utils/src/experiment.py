from geometry_msgs.msg import Wrench
from sensor_msgs import Joy
import pandas as pd
import pickle


class experiment:

	def __init__(self):
		self.teleopControl = False
		self.THRUST_MAX = sys.argv[0]
		self.duration = sys.argv[1]
		self.cameraPosition = None
		self.collectedData = {}
		self.message = Wrench()
	    thrusterCommand = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=10)
	    teleopCommand = rospy.Subscriber("/desiredThrustWrench", Joy, teleopCommand)
	    cameraPosition = rospy.Subscriber("/camera_position", CameraPosition, setCamera)

	def teleopCommand(self, data):
		self.teleopControl = Joy.axes[4]


	def setCamera(self, data):
		self.cameraPosition = (data.x, data.y)

	def experiment(self):
		if not self.teleopControl:
		    for thrust in np.linspace(0, self.THRUST_MAX, num=10):
		    	if not self.teleopControl:
			    	for direction in [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]:

			    	if not self.teleopControl:
				        message.force = thrust * direction

				    	try:
				    		if not self.teleopControl:
					    		startTime = time.time()
					    		startPosition = self.cameraPosition

					    		thrusterCommand.publish(message)

					    		while startTime + duration < time.time():
					    			if not self.teleopControl:
					    				break

					    		endPosition = self.cameraPosition

					    		self.collectedData[message.force] = endPosition - startPosition

def main(args):
    ex = experiment()
    rospy.init_node('experiment', anonymous=False)
    try:
        rospy.spin()
    	data = pd.from_dict(ex.collectedData)
    	pickle.dump(data, open( "experiment_data.p", "wb"))
    except KeyboardInterrupt():
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
