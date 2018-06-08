#!/user/bin/env python
import rospy
import tf
import actionlib
import actionlib_msgs.msg
import vision_manager.msg
import visual_servo.msg
import visual_servo as vs
from geometry_msgs.msg import Wrench

class locate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure', 'qualified'])
		self.vision_client = actionlib.SimpleActionClient('track_object')
		self.vision_client.wait_for_server()

	def execute(self, userdata):
		rospy.loginfo("Locating the gate.")
		start = rospy.Time(0)
		goal = vision_manager.msg.TrackObjectGoal()
		goal.objectType = goal.startGate
		self.vision_client.send_goal(goal)

		self.vision_client.wait_for_result()
		result = vision_client.get_result()

		if result.found:
			rospy.loginfo("Gate located.")
			return 'success'

		else:
			return 'failure'

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure', 'qualified'])
		self.servo_client = actionlib.SimpleActionClient('visual_servo')
        self.servo_client.wait_for_server()

	def execute(self, userdata):
		rospy.loginfo("Aligning the gate.")
		start = rospy.Time(0)
		goal = visual_servo.TrackObjectGoal()
		goal.objectType = goal.aligned
		self.servo_client.send_goal(goal)

		self.servo_client.wait_for_result()
		result = servo_client.get_result()

		if result.success:
			return 'success'
		
		else: 
			return 'failure'

class through(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure', 'qualified'])
		self.message = Wrench()
		self.vision_client = actionlib.SimpleActionClient('track_object')
		self.vision_client.wait_for_server()

	def execute(self, userdate):
		rospy.loginfo("Going through the gate")
		self.client.wait_for_result()
		result = vision_client.get_result()

		if not result.success:

			message.force.x = 1
			message.force.y = 0
			message.force.z = 0
			message.torque.x = 0
			message.torque.y = 0
			message.torque.z = 0
			return 'failure'

		else:
			#Todo: Figure out where to put this
			if qualStatus.poleDone:
				return 'qualified'
			else:
				return 'success'