#! /usr/bin/env python2
from enum import Enum
import math, time

from nav_msgs.msg import Odometry
from sub_trajectory.msg import StabilityMode
from geometry_msgs.msg import Wrench

from dynamic_reconfigure.server import Server
from sub_trajectory.cfg import StabilityConfig

import tf
import rospy
import numpy as np

def rosToArray(msg): #Convert a ros message with 1-4 dimensions into a numpy array
    return np.array([getattr(msg, key) for key in ("x", "y", "z", "w") if hasattr(msg, key)])

def rotateVector(quat, vec):
	quat2 = list(vec)
	quat2.append(0.0)
	return tf.transformations.quaternion_multiply(
		tf.transformations.quaternion_multiply(quat, quat2),
		tf.transformations.quaternion_conjugate(quat)
	)[:3]

class ActiveStabilizer():
	def __init__(self):
		self.curDepthMode = StabilityMode.off
		self.curAngleMode = StabilityMode.off
		self.yawEnabled = False

		self.saveDepth = True
		self.saveOrientation = True

		self.targetDepth = None
		self.targetOrientation = None

		self.lastUpdateTime = None

		self.depthVelGain = 1
		self.depthPosGain = [1, 0, 1]
		self.depthLastPos = None
		self.depthIntegratedError = 0
		self.depthMaxForce = 5

		self.orientVelGain = 1
		self.orientPosGain = [1, 0, 1]
		self.orientLastError = None
		self.orientIntegratedError = np.array([0,0,0])
		self.orientMaxForce = 2

		self.reconfigureServer = Server(StabilityConfig, self.reconfigureCallback)

		self.angleModeSubscriber = rospy.Subscriber("/thrusters/angleMode", StabilityMode, self.angleModeCallback, queue_size=1)
		self.depthModeSubscriber = rospy.Subscriber("/thrusters/depthMode", StabilityMode, self.depthModeCallback, queue_size=1)
		self.odomSubcriber = rospy.Subscriber("/odometry/filtered",Odometry, self.callback, queue_size=1)

		self.stabilityPublisher = rospy.Publisher("/stabilityWrench", Wrench, queue_size=1)

		self.stabilityWrench = Wrench()

	def reconfigureCallback(self, config, level):
		self.depthVelGain = config["depth_vel_P"]
		self.depthPosGain = [config["depth_pos_P"],
							config["depth_pos_I"],
							config["depth_pos_D"]]
		self.depthMaxForce = config["depth_pos_Max"]
		
		self.orientVelGain = config["orientation_vel_P"]
		self.orientPosGain = [config["orientation_pos_P"],
							config["orientation_pos_I"],
							config["orientation_pos_D"]]
		self.orientMaxForce = config["orientation_pos_Max"]

		return config

	def depthModeCallback(self, msg):
		self.curDepthMode = msg.mode

		if np.isfinite(msg.target.z):
			self.saveDepth = False
			self.targetDepth = msg.target.z
		elif self.curDepthMode != StabilityMode.position:
			self.saveDepth = True

	def angleModeCallback(self, msg):
		self.yawEnabled = msg.yawEnabled
		self.curAngleMode = msg.mode

		if np.isfinite(msg.target.w):
			self.saveOrientation = False
			self.targetOrientation = rosToArray(msg.target)
		if self.curAngleMode != StabilityMode.position:
			self.saveOrientation = True

	def callback(self, msg):
		timeNow = rospy.get_time()
		
		#TODO: How to reset the error integrations?
		if self.depthLastPos is None or self.curDepthMode is not StabilityMode.position:
			self.depthLastPos = msg.pose.pose.position.z
		
		if self.orientLastError is None or self.curAngleMode is not StabilityMode.position:
			self.orientLastError = np.array([0,0,0])
			self.orientIntegratedError = np.array([0,0,0])

		if self.curDepthMode == StabilityMode.off:
			self.stabilityWrench.force.x = 0
			self.stabilityWrench.force.y = 0
			self.stabilityWrench.force.z = 0
		
		elif self.curDepthMode == StabilityMode.velocity:
			quat = rosToArray(msg.pose.pose.orientation)
			counterVec = rotateVector(quat, [0,0,-self.depthVelGain*msg.twist.twist.linear.z])
			self.stabilityWrench.force.x = counterVec[0]
			self.stabilityWrench.force.y = counterVec[1]
			self.stabilityWrench.force.z = counterVec[2]

		elif self.curDepthMode == StabilityMode.position:
			if self.saveDepth:
				self.targetDepth = msg.pose.pose.position.z
				self.saveDepth = False
			error = msg.pose.pose.position.z - self.targetDepth
			
			proportionalCorrection = -self.depthPosGain[0]*error
			integralCorrection = 0
			derivativeCorrection = 0
			
			#TODO: Should we actually use the update time for this stuff?
			if self.lastUpdateTime is not None:
				#Depth derivative computation
				derivativeCorrection = -self.depthPosGain[2] * (msg.pose.pose.position.z - self.depthLastPos) / (timeNow - self.lastUpdateTime)
				self.depthLastPos = msg.pose.pose.position.z

				self.depthIntegratedError += error * (timeNow - self.lastUpdateTime)
				integralCorrection = -self.depthPosGain[1] * self.depthIntegratedError
				#Back-calculation integrator windup prevention
				if abs(proportionalCorrection + derivativeCorrection + integralCorrection) > self.depthMaxForce:
					self.depthIntegratedError = ((-np.sign(error)*self.depthMaxForce) - derivativeCorrection - proportionalCorrection)/-self.depthPosGain[1]
				integralCorrection = -self.depthPosGain[1] * self.depthIntegratedError

			#print([ proportionalCorrection, derivativeCorrection, integralCorrection])	

			quat = rosToArray(msg.pose.pose.orientation)
			quat = tf.transformations.quaternion_conjugate(quat)
			counterVec = rotateVector(quat, [0,0,proportionalCorrection + derivativeCorrection + integralCorrection])
			self.stabilityWrench.force.x = counterVec[0]
			self.stabilityWrench.force.y = counterVec[1]
			self.stabilityWrench.force.z = counterVec[2]

		else:
			self.stabilityWrench.force.x = 0
			self.stabilityWrench.force.y = 0
			self.stabilityWrench.force.z = 0

		if self.curAngleMode == StabilityMode.off:
			self.stabilityWrench.torque.x = 0
			self.stabilityWrench.torque.y = 0
			self.stabilityWrench.torque.z = 0
		
		elif self.curAngleMode == StabilityMode.velocity:
			self.stabilityWrench.torque.x = -self.orientVelGain * msg.twist.twist.angular.x
			self.stabilityWrench.torque.y = -self.orientVelGain * msg.twist.twist.angular.y

			if self.yawEnabled:
				self.stabilityWrench.torque.z = -self.orientVelGain * msg.twist.twist.angular.z
			else:
				self.stabilityWrench.torque.z = 0.0

		elif self.curAngleMode == StabilityMode.position:
			if self.saveOrientation:
				self.targetOrientation = rosToArray(msg.pose.pose.orientation)
				self.saveOrientation = False

			error = tf.transformations.quaternion_multiply(
				self.targetOrientation,
				tf.transformations.quaternion_conjugate(rosToArray(msg.pose.pose.orientation))
			) #Find rotation from current to target position

			#TODO: Do we need to transform the error and how
			#subLocalError = tf.transformations.quaternion_multiply(
			#	error, rosToArray(msg.pose.orientation)
			#) #Rotate rotation from current to target by rotation from world to sub

			rpyError = tf.transformations.euler_from_quaternion(error)

			proportionalCorrection = -self.orientPosGain[0]*rpyError
			integralCorrection = np.array([0,0,0])
			derivativeCorrection = np.array([0,0,0])
			
			#TODO: Should we actually use the update time for this stuff?
			if self.lastUpdateTime is not None:
				#Depth derivative computation
				derivativeCorrection = -self.orientPosGain[2] * (rpyError - self.orientLastError) / (timeNow - self.lastUpdateTime)
				self.orientLastError = rpyError

				self.orientIntegratedError += rpyError * (timeNow - self.lastUpdateTime)
				integralCorrection = -self.orientPosGain[1] * self.orientIntegratedError
				#Back-calculation integrator windup prevention
				for i in range(3):
					if abs(proportionalCorrection[i] + derivativeCorrection[i] + integralCorrection[i]) > self.orientMaxForce:
						self.orientIntegratedError[i] = ((-np.sign(rpyError[i])*self.orientMaxForce) - derivativeCorrection[i] - proportionalCorrection[i])/-self.orientPosGain[1]
				
				integralCorrection = -self.orientPosGain[1] * self.orientIntegratedError

			self.stabilityWrench.torque.x = proportionalCorrection[0] + derivativeCorrection[0] + integralCorrection[0]
			self.stabilityWrench.torque.y = proportionalCorrection[1] + derivativeCorrection[1] + integralCorrection[1]
			if self.yawEnabled:
				self.stabilityWrench.torque.z = proportionalCorrection[2] + derivativeCorrection[2] + integralCorrection[2]
			else:
				self.stabilityWrench.torque.z = 0.0

		else:
			self.stabilityWrench.torque.x = 0
			self.stabilityWrench.torque.y = 0
			self.stabilityWrench.torque.z = 0

		self.stabilityPublisher.publish(self.stabilityWrench)
		self.lastUpdateTime = timeNow

if __name__== "__main__":
	rospy.init_node('ActiveStability', anonymous=False)
	actStab = ActiveStabilizer()
	rospy.spin()
