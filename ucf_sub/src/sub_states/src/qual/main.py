#!/usr/bin/env python

import rospy
import smach
import smach_ros

import actionlib
import actionlib_msgs.msg

import vision_manager
import trajectory_planner

import vision_manager.msg
import trajectory_planner.msg

import monitor

import gate
import pole

class SubStates:

	def __init__(self):
		rospy.loginfo("State Machine has started.")

		self.movement_client = actionlib.SimpleActionClient('movement_server')
		rospy.loginfo("Created SimpleActionClient.")

                self.vision_client = actionlib.SimpleActionClient('track_object')
                rospy.loginfo("Created SimpleActionClient.")

		self.tasks = smach.StateMachine(outcomes=['preempted', 'success', 'failure', 'qualified'])

		with tasks:

			smach.StateMachine.add("GATE", gate(),
					       transitions={'preempted':'preempted',
							    'success':'POLE',
							    'failure':'GATE',
							    'qualified':'success'})

			smach.StateMachine.add("POLE", pole(),
					       transitions={'preempted':'preempted',
							    'success':'GATE',
							    'failure':'POLE',
							    'qualified': 'POLE'})

			gate = smach.StateMachine(outcomes=['preempted', 'success', 'failure', 'qualified'])

			with gate:
				smach.StateMachine.add('LOCATE', locate(),
						       transitions={'preempted':'preempted',
								    'success': 'ALIGN',
								    'failure': 'LOCATE',
								    'qualified': 'LOCATE'})

				smach.StateMachine.add('ALIGN', align(),
						       transitions={'preempted':'preempted',
								    'success': 'THROUGH',
								    'failure': 'LOCATE',
								    'qualified': 'LOCATE'})

				smach.StateMachine.add('THROUGH', through(),
						       transitions={'preempted':'preempted',
								    'success': 'POLE',
								    'failure':'LOCATE',
								    'qualified': 'success'})
			return gate

			pole = smach.StateMachine(outcomes=['preempted', 'success', 'failure'])

			with pole:

				smach.StateMachine.add('LOCATE', locate(),
						       transitions={'preempted':'preempted',
								    'success': 'ALIGN',
								    'failure': 'LOCATE'})

				smach.StateMachine.add('ALIGN', align(),
						       transitions={'preempted':'preempted',
								     'success': 'DRIFT',
								     'failure': 'LOCATE'})

				smach.StateMachine.add('DRIFT', drift(),
						       transitions={'preempted':'preempted',
								    'success': 'GATE',
								    'failure': 'LOCATE'})

		outcome = tasks.exectute()

if __name__ == '__main__':
	rospy.init_node('hippo_sm')
	sm = SubStates()
	rospy.spin()
