#!/user/bin/env python
import smach
import rospy
import time
import actionlib
from actionlib_msgs.msg import GoalStatus
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult, VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult
from geometry_msgs.msg import Wrench

class locate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
		self.client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
		self.client.wait_for_server()

	def execute(self, userdata):
		rospy.loginfo("Locating the gate.")
		start = rospy.Time(0)

		goal = TrackObjectGoal()
		goal.objectType = goal.startGate
		self.client.send_goal(goal)
		
		while self.client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING] and not self.preempt_requested():
			time.sleep(0.01)

		if self.preempt_requested():
			return 'preempted'
		result = self.client.get_result()

		if result.found:
			rospy.loginfo("Gate located.")
			return 'success'

		else:
			return 'failure'

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
		self.client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        	self.client.wait_for_server()

	def execute(self, userdata):
		rospy.loginfo("Aligning the gate.")
		start = rospy.Time(0)

		goal = VisualServoGoal()
		goal.servotask = goal.gate
		self.client.send_goal(goal)

		while self.client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING] and not self.preempt_requested():
			time.sleep(0.01)

		if self.preempt_requested():
			return 'preempted'

		result = self.client.get_result()

		if result.aligned:
			return 'success'

		else:
			return 'failure'

class through(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
		self.message = Wrench()
		self.client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
		self.client.wait_for_server()

	def execute(self, userdata):
		rospy.loginfo("Going through the gate")

		goal = TrackObjectGoal()
		goal.objectType = goal.startGate
		self.client.send_goal(goal)

		while self.client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING] and not self.preempt_requested():
			time.sleep(0.01)

		if self.preempt_requested():
			return 'preempted'

		result = self.client.get_result()

		if not result.found:
			return 'failure'

		else:
			return 'success'

def makeTask():
	task = smach.StateMachine(outcomes=['DONE'])
	with task:
		smach.StateMachine.add('LOCATE', locate(),
						transitions={'preempted':'DONE',
							'success': 'ALIGN',
							'failure': 'LOCATE'})

		smach.StateMachine.add('ALIGN', align(),
						transitions={'preempted':'DONE',
							'success': 'THROUGH',
							'failure': 'LOCATE'})

		smach.StateMachine.add('THROUGH', through(),
						transitions={'preempted':'DONE',
							'success': 'DONE',
							'failure':'LOCATE'})
	
	return task