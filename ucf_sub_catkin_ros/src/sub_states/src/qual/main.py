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

def main():
	rospy.init_node('hippo_sm')
	rospy.loginfo("State Machine has started.")

	mission_sm = smach.StateMachine(outcomes=['preempted', 'DONE', 'ABORT'])

	with mission_sm:

		smach.StateMachine.add('Start', StartState(), transitions={'GO':'GATE'})

		task_gate = smach.StateMachine(outcomes=['preempted', 'DONE', 'ABORT'])
		with task_gate:
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
		
		task_pole = smach.StateMachine(outcomes=['preempted', 'DONE', 'ABORT'])
		with task_pole:
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

		smach.StateMachine.add('GATE', task_gate, transitions={'preempted':'preempted','DONE':'POLE'})
		smach.StateMachine.add('POLE', task_pole, transitions={'preempted':'preempted','DONE':'DONE'})

	sis = smach_ros.IntrospectionServer("sm_server", mission_sm, "/MISSION")
	sis.start()
	mission_sm.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
