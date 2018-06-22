#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
class ThrusterControl:

	def __init__(self):
		self.trans_sub = rospy.Subscriber("/translate/joy", Joy, self.translateCb)
		self.rot_sub = rospy.Subscriber("/rotate/joy", Joy, self.rotateCb)
		self.twist_pub = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=1000)
		self.limit_pub = rospy.Publisher("/thruster_limit", Float32, queue_size=1000)
		self.autonomyEnabled = 1
		self.autonomy_sub = rospy.Subscriber("/autonomyWrench", Wrench, self.republishWrench)
		self.autonomy_pub = rospy.Publisher("/autonomyWrench", Wrench, queue_size=1000)
		self.twistMsg = Wrench()

	def translateCb(self, msg):
		if(len(msg.axes) < 3):
			rospy.loginfo("JOYSTICK ERROR")
			return

		self.twistMsg.force.x = msg.axes[1] * 20
		self.twistMsg.force.y = msg.axes[0] * 20
		self.twistMsg.force.z = msg.axes[2] * 20
		thruster_limit = (msg.axes[3]+1)/4
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
		rospy.loginfo("ROTATION UPDATED")


	def republishWrench(self, msg):
		if self.autonomyEnabled > 0:
			self.twist_pub.publish(msg)

if __name__== "__main__":
	tc = ThrusterControl()
	rospy.init_node('ThrusterControl', anonymous=False)
	rate = rospy.Rate(50)

	while not rospy.is_shutdown():
		if tc.autonomyEnabled < 0:
			tc.twist_pub.publish(tc.twistMsg)
		rate.sleep()
