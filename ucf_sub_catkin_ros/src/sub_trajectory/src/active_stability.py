from enum import Enum
from nav_msgs.msg import Odometry
from sub_trajectory.msg import StabilityMode
import tf
import rospy

class depthModes(Enum):
	off = 1
	velocity = 2
	position = 3

class angleModes(Enum):
	off = 1
	velocity = 2
	position = 3
	yawEnabled = 4


class active_stabilization(self):
	def __init__(self):
		self.curDepthMode = depthModes.off
		self.curAngleMode = angleModes.off
		self.yawEnabled == False
		self.angleModeSubscriber = rospy.Subscriber("/thrusters/angleMode", StabilityMode, self.depthModeCallback, queue_size=1)
		self.depthModeSubscriber = rospy.Subscriber("/thrusters/depthMode", StabilityMode, self.angleModeCallback, queue_size=1)
		self.odomSubcriber = rospy.Subscriber("/odometry/local",Odometry, self.callback, queue_size=1)
		self.stabilityWrench = Wrench()
		self.yawEnabled == False
		listener = tf.TransformListener()

	def depthModeCallback(self, msg):
		self.curDepthMode = msg.depthMode

	def angleModeCallback(self,msg):
		if msg.yawEnabled is not self.yawEnabled:
			self.yawEnabled = msg.yawEnabled
		
		else:
			self.curAngleMode = msg.angleMode

	def callback(self, msg):
		self.pose = msg.pose
		self.twist = msg.twist

		if self.curDepthMode == depthModes.off.value:
			self.stabilityWrench.force.x = newStability
			self.stabilityWrench.force.y = newStability
			self.stabilityWrench.force.z = newStability
		
		elif self.curDepthMode == depthModes.velocity.value:
			unTransformed = - msg.twist.linear.z
			newStability = listener.transformVector3(base_link, unTransformed)
			self.stabilityWrench.force.x = newStability
			self.stabilityWrench.force.y = newStability
			self.stabilityWrench.force.z = newStability

		else:
			orientQuaion = listener.transformDATATYPE(base_link, msg.pose)


		if self.curAngleMode == angleModes.off.value:
			self.stabilityWrench.force.x = newStability
			self.stabilityWrench.force.y = newStability
			self.stabilityWrench.force.z = newStability	
		
		elif self.curAngleMode == angleModes.velocity.value:
			self.stabilityWrench.torque.x = -msg.twist.angular * 10
			self.stabilityWrench.torque.y = -msg.twist.angular * 10

			if not self.yawEnabled:
				self.stabilityWrench.torque.z = -msg.twist.angular 			

		elif self.curAngleMode == angleModes.position.value:
			pass

		else:
			pass		