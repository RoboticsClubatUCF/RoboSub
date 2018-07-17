#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool
import smach, smach_ros

import gate
import pole

class StartState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['GO'])
		self.waiting = True
		rospy.Subscriber("/start", Bool, self.go)
	
	def go(self, msg):
		self.waiting = False
	
	def execute(self, userdata):
		while self.waiting:
			time.sleep(0.01)

		return 'GO'

		
class SubStates:
	def __init__(self):
		rospy.loginfo("State Machine has started.")

		self.gate = smach.StateMachine(outcomes=['preempted', 'DONE', 'ABORT'])
		self.pole = smach.StateMachine(outcomes=['preempted', 'DONE', 'ABORT'])
		self.tasks = smach.StateMachine(outcomes=['preempted', 'DONE', 'ABORT'])

		with self.tasks:

			smach.StateMachine.add('Start', StartState(), transitions={'GO':'GATE'})

			with self.gate:
				smach.StateMachine.add('LOCATE', gate.locate(),
						       transitions={'preempted':'preempted',
								    'success': 'ALIGN',
								    'failure': 'LOCATE'})

				smach.StateMachine.add('ALIGN', gate.align(),
						       transitions={'preempted':'preempted',
								    'success': 'THROUGH',
								    'failure': 'LOCATE'})

				smach.StateMachine.add('THROUGH', gate.through(),
						       transitions={'preempted':'preempted',
								    'success': 'DONE',
								    'failure':'LOCATE'})
			with self.pole:
				smach.StateMachine.add('LOCATE', pole.locate(),
						       transitions={'preempted':'preempted',
								    'success': 'ALIGN',
								    'failure': 'LOCATE'})

				smach.StateMachine.add('ALIGN', pole.align(),
						       transitions={'preempted':'preempted',
								     'success': 'DRIFT',
								     'failure': 'LOCATE'})

				smach.StateMachine.add('DRIFT', pole.drift(),
						       transitions={'preempted':'preempted',
								    'success': 'DONE',
								    'failure': 'LOCATE'})

			smach.StateMachine.add('GATE', self.gate, transitions={'preempted':'preempted','DONE':'POLE'})
			smach.StateMachine.add('POLE', self.pole, transitions={'preempted':'preempted','DONE':'DONE'})

if __name__ == '__main__':
	rospy.init_node('hippo_sm')
	sm = SubStates()
	sis = smach_ros.IntrospectionServer("sm_server", sm.tasks, "/MAIN")
	sis.start()
	outcome = sm.tasks.execute()
	rospy.spin()
	sis.stop()
