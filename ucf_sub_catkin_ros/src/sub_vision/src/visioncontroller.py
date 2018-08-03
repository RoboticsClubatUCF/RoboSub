import rospy
from geometry_msgs.msg import WrenchStamped
from dynamic_reconfigure.server import Server
from sub_vision.cfg import ServoingConfig
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult

class VisionController:

	def __init__(self):
		self.thruster_pub = rospy.Publisher('/autonomyWrench', WrenchStamped, queue_size=1)
		self.thruster_command = WrenchStamped()
		self.vision_feedback = rospy.Subscriber('/track_object/feedback', TrackObjectFeedback, self.execute)
		self.reconfigureServer = Server(ServoingConfig, self.reconfigureCallback)
		self.coeff = 1

	def reconfigureCallback(self, config, level):
		self.coeff = config["lam"]
		return config

	def execute(self, msg):
		if msg.center[0]-msg.size[0]/2 < 0:
			self.thruster_command.y = -1 * self.coeff
			rospy.loginfo(self.thruster_command.y)
			self.thruster_pub.publish(self.thruster_command)

		elif msg.center[0]-msg.size[0]/2 > 0:
			self.thruster_command.y = 1 * self.coeff
			rospy.loginfo(self.thruster_command.y)
			self.thruster_pub.publish(self.thruster_command)

		else:
			self.thruster_command.y = 0
			rospy.loginfo(self.thruster_command.y)
			self.thruster_pub.publish(self.thruster_command)

if __name__== "__main__":
    rospy.init_node('vision_controller', anonymous=False)
    vs = VisionController()
    rospy.spin()
