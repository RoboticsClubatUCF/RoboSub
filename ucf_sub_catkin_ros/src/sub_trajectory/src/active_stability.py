#! /usr/bin/env python
from enum import Enum
from nav_msgs.msg import Odometry
from sub_trajectory.msg import StabilityMode
import tf
import rospy
import numpy as np

class modes(Enum):
	off = 1
	velocity = 2
	position = 3

def rosToArray(msg): #Convert a ros message with 1-4 dimensions into a numpy array
    return np.array([getattr(msg, key) for key in ("x", "y", "z", "w") if hasattr(msg, key)])

def rotateVector(quat, vec):
	quat2 = list(vec).append(0.0)
	return tf.transformations.quaternion_multiply(
		tf.transformations.quaternion_multiply(quat, quat2),
		tf.transformations.quaternion_conjugate(quat)
	)[:3]

class ActiveStabilizer():
	def __init__(self):
		self.curDepthMode = modes.off
		self.curAngleMode = modes.off
		self.yawEnabled == False

		self.saveDepth = True
		self.saveOrientation = True

		self.targetDepth = None
		self.targetOrientation = None

		self.angleModeSubscriber = rospy.Subscriber("/thrusters/angleMode", StabilityMode, self.depthModeCallback, queue_size=1)
		self.depthModeSubscriber = rospy.Subscriber("/thrusters/depthMode", StabilityMode, self.angleModeCallback, queue_size=1)
		self.odomSubcriber = rospy.Subscriber("/odometry/local",Odometry, self.callback, queue_size=1)

		self.stabilityPublisher = rospy.Publisher("/stabiltyWrench", Wrench, queue_size=1)

		self.stabilityWrench = Wrench()

	def depthModeCallback(self, msg):
		self.curDepthMode = msg.depthMode
		if self.curDepthMode != modes.position.value:
			self.saveDepth = True

	def angleModeCallback(self, msg):
		self.yawEnabled = msg.yawEnabled
		self.curAngleMode = msg.Mode
		if self.curAngleMode != modes.position.value:
			self.saveOrientation = True

	def callback(self, msg):
		self.pose = msg.pose
		self.twist = msg.twist

		if self.curDepthMode == modes.off.value:
			self.stabilityWrench.force.x = 0
			self.stabilityWrench.force.y = 0
			self.stabilityWrench.force.z = 0
		
		elif self.curDepthMode == modes.velocity.value:
			quat = rosToArray(msg.pose.orientation)
			counterVec = rotateVector(quat, [0,0,-10*msg.twist.linear.z])
			self.stabilityWrench.force.x = counterVec[0]
			self.stabilityWrench.force.y = counterVec[1]
			self.stabilityWrench.force.z = counterVec[2]

		elif self.curDepthMode == modes.position.value:
			if self.saveDepth:
				self.targetDepth = msg.pose.position.z
			error = msg.pose.position.z - self.targetDepth
			quat = rosToArray(msg.pose.orientation)
			counterVec = rotateVector(quat, [0,0,-10*error])
			self.stabilityWrench.force.x = counterVec[0]
			self.stabilityWrench.force.y = counterVec[1]
			self.stabilityWrench.force.z = counterVec[2]

		else:
			self.stabilityWrench.force.x = 0
			self.stabilityWrench.force.y = 0
			self.stabilityWrench.force.z = 0

		if self.curAngleMode == modes.off.value:
			self.stabilityWrench.torque.x = 0
			self.stabilityWrench.torque.y = 0
			self.stabilityWrench.torque.z = 0
		
		elif self.curAngleMode == modes.velocity.value:
			self.stabilityWrench.torque.x = -msg.twist.angular.x * 10
			self.stabilityWrench.torque.y = -msg.twist.angular.y * 10

			if self.yawEnabled:
				self.stabilityWrench.torque.z = -msg.twist.angular.z 			

		elif self.curAngleMode == modes.position.value:
			if self.saveOrientation:
				self.targetOrientation = rosToArray(msg.pose.orientation)

			error = tf.transformations.quaternion_multiply(
				self.targetOrientation,
				tf.transformations.quaternion_conjugate(rosToArray(msg.pose.orientation))
			) #Find rotation from current to target position

			#TODO: Do we need to transform the error and how
			#subLocalError = tf.transformations.quaternion_multiply(
			#	error, rosToArray(msg.pose.orientation)
			#) #Rotate rotation from current to target by rotation from world to sub

			rpy = tf.transformations.euler_from_quaternion(error)
			self.stabilityWrench.torque.x = rpy[0]
			self.stabilityWrench.torque.y = rpy[1]
			self.stabilityWrench.torque.z = rpy[2]

		else:
			self.stabilityWrench.torque.x = 0
			self.stabilityWrench.torque.y = 0
			self.stabilityWrench.torque.z = 0

		self.stabilityPublisher.publish(self.stabilityWrench)

if __name__== "__main__":
	actStab = ActiveStabilizer()
	rospy.init_node('ActiveStability', anonymous=False)
	rospy.spin()
