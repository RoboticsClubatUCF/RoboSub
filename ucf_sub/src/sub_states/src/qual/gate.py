#!/user/bin/env python
import rospy
import tf
import actionlib
import actionlib_msgs.msg
import vision_manager.msg
import visual_servo as vs
from geometry_msgs.msg import Wrench

class locate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure', 'qualified'])
		self.vision_client = actionlib.SimpleActionClient('track_object')
		self.vision_client.wait_for_server()
		self.listener = tf.TransformListener()
	def execute(self, userdata):
		rospy.loginfo("Locating the gate.")
		start = rospy.Time(0)
		found = vision_manager.msg.TrackObjectResult()
		goal = vision_manager.msg.TrackObjectGoal()
		goal.objectType = goal.startGate
		self.vision_client.send_goal(goal)

		if found:
			rospy.loginfo("Gate located.")
			return 'success'
		if not found:
			return failure

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure', 'qualified'])
	def execute(self, userdata):
		rospy.loginfo("Aligning the gate.")
		vs.main()
		if vs.aligned:
			return 'success'
		else:
			return 'failure'

class through(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure', 'qualified'])
		self.message = Wrench()

	def execute(self, userdate):
		rospy.loginfo("Going through the gate")

		while vision_manager.msg.TrackObjectResult().found:
			message.force.x = 1
			message.force.y = 0
			message.force.z = 0
			message.torque.x = 0
			message.torque.y = 0
			message.torque.z = 0

		qualStatus = vision_manager.msg.TrackObjectResult()

		if qualStatus.poleDone:
			return 'qualified'
		else:
			return 'success'
