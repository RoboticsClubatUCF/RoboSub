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

	def execute(self, userdata):
		pass

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])

	def execute(self, userdata):
		pass

class pull(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])

	def execute(self, userdata):
		pass

class backup(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])

	def execute(self, userdata):
		pass

class ute(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure', 'done'])

	def execute(self, userdata):
		pass


def makeTask():
	task = smach.StateMachine(outcomes=['DONE'])
	with task:
		smach.StateMachine.add('LOCATE', locate(),
						transitions={'preempted':'DONE',
							'success': 'ALIGN',
							'failure': 'LOCATE'})

		smach.StateMachine.add('ALIGN', align(),
						transitions={'preempted':'DONE',
							'success': 'PULL',
							'failure': 'LOCATE'})

		smach.StateMachine.add('PULL', pull(),
						transitions={'preempted':'DONE',
							'success': 'BACKUP',
							'failure':'PULL'})
	
		smach.StateMachine.add('BACKUP', backup(),
						transitions={'preempted':'DONE',
							'success': 'FINDHOLE',
							'failure':'BACKUP'})

		smach.StateMachine.add('UTE', ute(),
						transitions={'preempted':'DONE',
							'success': 'BACKUP',
							'failure':'UTE',
							'done': 'DONE'})		

	return task