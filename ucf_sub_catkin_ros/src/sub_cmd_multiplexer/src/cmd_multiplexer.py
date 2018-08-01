#! /usr/bin/env python3
import rospy

from geometry_msgs.msg import Wrench, WrenchStamped
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool

from sub_trajectory.msg import StabilityMode

from enum import Enum
class modes(Enum):
	off = 1
	velocity = 2
	position = 3

class ThrusterControl:

	def __init__(self):
		self.twistMsg = WrenchStamped()
		self.stabilityMsg = WrenchStamped()
		self.depthMode = modes.off
		self.angleMode = modes.off
		self.yawEnabled = False
		self.stabilityMode = StabilityMode()
		self.autonomyEnabled = -1

		self.trans_sub = rospy.Subscriber("/translate/joy", Joy, self.translateCb)
		self.rot_sub = rospy.Subscriber("/rotate/joy", Joy, self.rotateCb)

		self.twist_pub = rospy.Publisher("/desiredThrustWrench", WrenchStamped, queue_size=10)
		self.limit_pub = rospy.Publisher("/thruster_limit", Float32, queue_size=10)

		self.autonomy_sub = rospy.Subscriber("/autonomyWrench", WrenchStamped, self.republishWrench)
		self.stability_sub = rospy.Subscriber("/stabilityWrench", WrenchStamped, self.stabilityWrench)

		self.depth_mode_pub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
		self.angle_mode_pub = rospy.Publisher("/thrusters/angleMode", StabilityMode, queue_size=1)

		self.torpedo1_pub = rospy.Publisher("/Torpedo1", Bool, queue_size=10)
		self.torpedo2_pub = rospy.Publisher("/Torpedo2", Bool, queue_size=10)

	def translateCb(self, msg):
		if(len(msg.axes) < 5):
			rospy.logerr("JOYSTICK ERROR: Not enough axes")
			return

		self.twistMsg.wrench.force.x = msg.axes[1] * 7
		self.twistMsg.wrench.force.y = msg.axes[0] * -7
		self.twistMsg.wrench.force.z = msg.axes[2] * -7

		thruster_limit = (msg.axes[3]+1)/4
		self.limit_pub.publish(thruster_limit)
		self.stabilityMode.target.z = float("NaN")
		if msg.axes[4] > 0.5 and abs(msg.axes[5]) < 0.5:
			self.stabilityMode.mode = self.stabilityMode.velocity
			self.depth_mode_pub.publish(self.stabilityMode)

		elif abs(msg.axes[4]) < 0.5 and msg.axes[5] > 0.5:
			self.stabilityMode.mode = self.stabilityMode.position
			self.depth_mode_pub.publish(self.stabilityMode)

		elif abs(msg.axes[4]) < 0.5 and msg.axes[5] < -0.5:
			self.stabilityMode.mode = self.stabilityMode.off
			self.depth_mode_pub.publish(self.stabilityMode)

		self.torpedo1_pub.publish(Bool(msg.buttons[0] > 0.5 and msg.buttons[1] > 0.5))
		rospy.logdebug("TRANSLATION UPDATED")


	def rotateCb(self, msg):
		if(len(msg.axes) < 5):
			rospy.logerr("JOYSTICK ERROR: Not enough axes")
			return
		self.twistMsg.wrench.torque.x = msg.axes[0] * -0.7
		self.twistMsg.wrench.torque.y = msg.axes[1] * -1
		self.twistMsg.wrench.torque.z = msg.axes[2] * -1
		self.autonomyEnabled = msg.axes[3]
		self.stabilityMode.target.w = float("NaN")
		if msg.axes[4] > 0.5 and abs(msg.axes[5]) < 0.5:
			self.stabilityMode.mode = self.stabilityMode.velocity
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.angle_mode_pub.publish(self.stabilityMode)

		elif msg.axes[4] < -0.5 and abs(msg.axes[5]) < 0.5:
			self.yawEnabled = not self.yawEnabled
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.angle_mode_pub.publish(self.stabilityMode)

		elif abs(msg.axes[4]) < 0.5 and msg.axes[5] > 0.5:
			self.stabilityMode.mode = self.stabilityMode.position
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.angle_mode_pub.publish(self.stabilityMode)

		elif abs(msg.axes[4]) < 0.5 and msg.axes[5] < -0.5:
			self.stabilityMode.mode = self.stabilityMode.off
			self.stabilityMode.yawEnabled = self.yawEnabled
			self.angle_mode_pub.publish(self.stabilityMode)
		self.torpedo2_pub.publish(Bool(msg.buttons[0] > 0.5 and msg.buttons[1] > 0.5))
		rospy.logdebug("ROTATION UPDATED")

	def republishWrench(self, msg):
		if self.autonomyEnabled > 0:
			msg.wrench.force.x = msg.wrench.force.x + self.stabilityMsg.wrench.force.x
			msg.wrench.force.y = msg.wrench.force.y + self.stabilityMsg.wrench.force.y
			msg.wrench.force.z = msg.wrench.force.z + self.stabilityMsg.wrench.force.z
			msg.wrench.torque.x = msg.wrench.torque.x + self.stabilityMsg.wrench.torque.x
			msg.wrench.torque.y = msg.wrench.torque.y + self.stabilityMsg.wrench.torque.y
			msg.wrench.torque.z = msg.wrench.torque.z + self.stabilityMsg.wrench.torque.z
			self.twist_pub.publish(msg)

	def stabilityWrench(self, msg):
		self.stabilityMsg = msg

if __name__== "__main__":
	rospy.init_node('ThrusterControl', anonymous=False)
	tc = ThrusterControl()
	rate = rospy.Rate(50)
	msg = WrenchStamped()
	msg.header.frame_id = 'base_link'
	while not rospy.is_shutdown():
		if tc.autonomyEnabled < 0:
			msg.wrench.force.x = tc.twistMsg.wrench.force.x + tc.stabilityMsg.wrench.force.x
			msg.wrench.force.y = tc.twistMsg.wrench.force.y + tc.stabilityMsg.wrench.force.y
			msg.wrench.force.z = tc.twistMsg.wrench.force.z + tc.stabilityMsg.wrench.force.z
			msg.wrench.torque.x = tc.twistMsg.wrench.torque.x + tc.stabilityMsg.wrench.torque.x
			msg.wrench.torque.y = tc.twistMsg.wrench.torque.y + tc.stabilityMsg.wrench.torque.y
			msg.wrench.torque.z = tc.twistMsg.wrench.torque.z + tc.stabilityMsg.wrench.torque.z
			msg.header.stamp = rospy.Time.now()	
			tc.twist_pub.publish(msg)
		rate.sleep()
