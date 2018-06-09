import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

class ThrusterControl:

	def __init__(self):
		self.trans_sub = rospy.Subscriber("/translate/joy", Joy, translateCb)
		self.rot_sub = rospy.Subscriber("/rotate/joy", Joy, rotateCb)
		self.twist_pub = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=1000)
		self.limit_pub = rospy.Publisher("/thruster_limit", Float32, queue_size=1000)
		self.autonomyEnabled = -1
		self.autonomy_sub = rospy.Subscriber("/autonomyWrench", Wrench, republishWrench)
		self.autonomy_pub = rospy.Publisher("/autonomyWrench", Wrench, queue_size=1000)
		self.twistmsg = Wrench()

	def translateCb(self, msg):
		if(len(msg.axes)/len(float) < 3):
			rospy.loginfo("JOYSTICK ERROR")
			return

		twistMsg.force.x = msg.axes[1] * 20
		twistMsg.force.y = msg.axes[0] * 20
		twistMsg.force.z = msg.axes[2] * 20
		thruster_limit = (msg.axes[3]+1)/4
		self.limit_pub.publish(thruster_limit)
		rospy.loginfo("TRANSLATION UPDATED")


	def rotateCb(self, msg):
		if(len(msg.axes)/len(float) < 3):
			rospy.loginfo("JOYSTICK ERROR: Not enough axes")
			return

		twistMsg.torque.x = msg.axes[0] * 20
		twistMsg.torque.y = msg.axes[1] * 20
		twistMsg.torque.z = msg.axes[2] * 20
		self.autonomyEnabled = msg.axes[3]
		rospy.loginfo("ROTATION UPDATED")


	def republishWrench(self, msg):
		if autonomyEnabled > 0:
			self.twist_pub.publish(msg)

if __name__== "__main__":
	tc = ThrusterControl()
	rospy.init_node('ThrusterControl', anonymous=False)
	rate = rospy.Rate(50)

	while not rospy.is_shutdown(:
		tc.twist_pub.publish(tc.twistMsg)
		rate.sleep()