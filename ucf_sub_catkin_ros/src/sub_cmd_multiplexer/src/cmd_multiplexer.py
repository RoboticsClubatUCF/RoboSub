#! /usr/bin/env python3
import rospy

from geometry_msgs.msg import Wrench
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
		self.twistMsg = Wrench()
		self.stabilityMsg = Wrench()
		self.depthMode = modes.off
		self.angleMode = modes.off
		self.yawEnabled = False
		self.stabilityMode = StabilityMode()
		self.autonomyEnabled = -1

		self.trans_sub = rospy.Subscriber("/translate/joy", Joy, self.translateCb)
		self.rot_sub = rospy.Subscriber("/rotate/joy", Joy, self.rotateCb)

		self.twist_pub = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=1000)
		self.limit_pub = rospy.Publisher("/thruster_limit", Float32, queue_size=1000)

		self.autonomy_sub = rospy.Subscriber("/autonomyWrench", Wrench, self.republishWrench)
		self.stability_sub = rospy.Subscriber("/stabilityWrench", Wrench, self.stabilityWrench)

		self.depth_mode_pub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
		self.angle_mode_pub = rospy.Publisher("/thrusters/angleMode", StabilityMode, queue_size=1)

		self.torpedo1_pub = rospy.Publisher("/Torpedo1", Bool, queue_size=10)
		self.torpedo2_pub = rospy.Publisher("/Torpedo2", Bool, queue_size=10)

	def translateCb(self, msg):
		if(len(msg.axes) < 5):
			rospy.logerror("JOYSTICK ERROR: Not enough axes")
			return

		self.twistMsg.force.x = msg.axes[1] * 7
		self.twistMsg.force.y = msg.axes[0] * 7
		self.twistMsg.force.z = msg.axes[2] * 7
		
		thruster_limit = (msg.axes[3]+1)/4
		self.limit_pub.publish(thruster_limit)

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
			rospy.logerror("JOYSTICK ERROR: Not enough axes")
			return
		self.twistMsg.torque.x = msg.axes[0] * -7
		self.twistMsg.torque.y = msg.axes[1] * -7
		self.twistMsg.torque.z = msg.axes[2] * -7
		self.autonomyEnabled = msg.axes[3]

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
			msg.force.x = msg.force.x + self.stabilityMsg.force.x
			msg.force.y = msg.force.y + self.stabilityMsg.force.y
			msg.force.z = msg.force.z + self.stabilityMsg.force.z
			msg.torque.x = msg.torque.x + self.stabilityMsg.torque.x
			msg.torque.y = msg.torque.y + self.stabilityMsg.torque.y
			msg.torque.z = msg.torque.z + self.stabilityMsg.torque.z
			self.twist_pub.publish(msg)

	def stabilityWrench(self, msg):
		self.stabilityMsg = msg

if __name__== "__main__":
	rospy.init_node('ThrusterControl', anonymous=False)
	tc = ThrusterControl()
	rate = rospy.Rate(50)
	msg = Wrench()
	while not rospy.is_shutdown():
		if tc.autonomyEnabled < 0:
			msg.force.x = tc.twistMsg.force.x + tc.stabilityMsg.force.x
			msg.force.y = tc.twistMsg.force.y + tc.stabilityMsg.force.y
			msg.force.z = tc.twistMsg.force.z + tc.stabilityMsg.force.z
			msg.torque.x = tc.twistMsg.torque.x + tc.stabilityMsg.torque.x
			msg.torque.y = tc.twistMsg.torque.y + tc.stabilityMsg.torque.y
			msg.torque.z = tc.twistMsg.torque.z + tc.stabilityMsg.torque.z

			tc.twist_pub.publish(msg)
		rate.sleep()
