#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from sub_trajectory.msg import StabilityMode


class ThrusterControl:

	def __init__(self):
		self.trans_sub = rospy.Subscriber("/translate/joy", Joy, self.translateCb)
		self.rot_sub = rospy.Subscriber("/rotate/joy", Joy, self.rotateCb)
		self.twist_pub = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=1000)
		self.limit_pub = rospy.Publisher("/thruster_limit", Float32, queue_size=1000)
		self.autonomyEnabled = 1
		self.autonomy_sub = rospy.Subscriber("/autonomyWrench", Wrench, self.republishWrench)
		self.autonomy_pub = rospy.Publisher("/autonomyWrench", Wrench, queue_size=1000)

		self.stability_pub = rospy.Subscriber("/stabilityWrench", Wrench, self.stabilityWrench)

		self.twistMsg = Wrench()
		self.stabilityMsg = Wrench()
		self.depthMode = depthModes.off
		self.angleMode = angleModes.off
		self.yawEnabled == False
		self.stabilityMode = StabilityMode()
		self.depth_mode_pub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
		self.angle_mode_pub = rospy.Publisher("/thrusters/angleMode", StabilityMode, queue_size=1)

	def translateCb(self, msg):
		if(len(msg.axes) < 3):
			rospy.loginfo("JOYSTICK ERROR")
			return

		self.twistMsg.force.x = msg.axes[1] * 20
		self.twistMsg.force.y = msg.axes[0] * 20
		self.twistMsg.force.z = msg.axes[2] * 20
		
		thruster_limit = (msg.axes[3]+1)/4

		if msg.axes[4] == 1 and msg.axes[5] == 0:
			self.stabilityMode.mode = self.stabilityMode.mode.velocity
			self.depth_mode_pub.publish(self.stabilityMode)

		elif msg.axes[4] == 0 and msg.axes[5] == 1:
			self.stabilityMode.mode = self.stabilityMode.mode.position
			self.depth_mode_pub.publish(self.stabilityMode)

		elif msg.axes[4] == 0 and msg.axes[5] == -1:
			self.stabilityMode.mode = self.stabilityMode.mode.off
			self.depth_mode_pub.publish(self.stabilityMode)


		self.limit_pub.publish(thruster_limit)
		rospy.loginfo("TRANSLATION UPDATED")


	def rotateCb(self, msg):
		if(len(msg.axes) < 3):
			rospy.loginfo("JOYSTICK ERROR: Not enough axes")
			return
		self.twistMsg.torque.x = msg.axes[0] * 20
		self.twistMsg.torque.y = msg.axes[1] * 20
		self.twistMsg.torque.z = msg.axes[2] * 20
		self.autonomyEnabled = msg.axes[3]

		if msg.axes[4] == 1 and msg.axes[5] == 0:
			self.stabilityMode.mode = self.stabilityMode.mode.velocity
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.angle_mode_pub.publish(self.stabilityMode)

		elif msg.axes[4] == -1 and msg.axes[5] == 0:
			self.stabilityMode.yawEnabled = not self.yawEnabled
			self.depth_mode_pub.publish(self.stabilityMode)

		elif msg.axes[4] == 0 and msg.axes[5] == 1:
			self.stabilityMode.mode = self.stabilityMode.mode.position
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.depth_mode_pub.publish(self.stabilityMode.mode.position)

		elif msg.axes[4] == 0 and msg.axes[5] == -1:
			self.stabilityMode.mode = self.stabilityMode.mode.off
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.depth_mode_pub.publish(self.stabilityMode.mode.off)

		rospy.loginfo("ROTATION UPDATED")


	def republishWrench(self, msg):
		if self.autonomyEnabled > 0:
			self.msg.force.x = self.msg.force.x + self.stabilityMsg.force.x
			self.msg.force.y = self.msg.force.y + self.stabilityMsg.force.y
			self.msg.force.z = self.msg.force.z + self.stabilityMsg.force.z
			self.msg.torque.x = self.msg.torque.x + self.stabilityMsg.torque.x
			self.msg.torque.y = self.msg.torque.x + self.stabilityMsg.torque.y
			self.msg.torque.z = self.msg.torque.x + self.stabilityMsg.torque.z
			self.twist_pub.publish(msg)

	def stabilityWrench(self, msg):
		stabilityMsg = msg

if __name__== "__main__":
	tc = ThrusterControl()
	rospy.init_node('ThrusterControl', anonymous=False)
	rate = rospy.Rate(50)

	while not rospy.is_shutdown():
		if tc.autonomyEnabled < 0:
			tc.twistMsg.force.x = tc.twistMsg.force.x + tc.stabilityMsg.force.x
			tc.twistMsg.force.y = tc.twistMsg.force.y + tc.stabilityMsg.force.y
			tc.twistMsg.force.z = tc.twistMsg.force.z + tc.stabilityMsg.force.z
			tc.twistMsg.torque.x = tc.twistMsg.torque.x + tc.stabilityMsg.torque.x
			tc.twistMsg.torque.y = tc.twistMsg.torque.y + tc.stabilityMsg.torque.y
			tc.twistMsg.torque.z = tc.twistMsg.torque.z + tc.stabilityMsg.torque.z

			tc.twist_pub.publish(tc.twistMsg)
		rate.sleep()
