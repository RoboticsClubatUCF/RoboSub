#!/usr/bin/env python

import rospy
import smach
import smach_ros

import actionlib
import actionlib_msgs.msg

#import monitor
import gate
import pole

class SubStates:

	def __init__(self):
		rospy.loginfo("State Machine has started.")

		self.gate = smach.StateMachine(outcomes=['preempted', 'POLE', 'GATE'])
		self.pole = smach.StateMachine(outcomes=['preempted', 'GATE', 'POLE'])
		self.tasks = smach.StateMachine(outcomes=['POLE', 'GATE', 'preempted', self.gate, self.pole])

		with self.tasks:

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
								    'success': 'POLE',
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
								    'success': 'GATE',
								    'failure': 'LOCATE'})

			smach.StateMachine.add('Start', self.gate, transitions={'POLE':self.pole, 'GATE':self.gate})
		

if __name__ == '__main__':
	rospy.init_node('hippo_sm')
	sm = SubStates()
	outcome = sm.tasks.execute()
	rospy.spin()
