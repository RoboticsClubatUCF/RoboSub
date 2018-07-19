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
		while self.waiting and not self.preempt_requested():
			time.sleep(0.1)
		
		return 'GO'

class SafetyState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ABORT', 'RECOVERED'])
		self.leak = False
		rospy.Subscriber("/leak", Bool, self.leakCb)

	def leakCb(self, msg):
		self.leak = msg.data
	
	def execute(self, userdata):
		while not self.leak and not self.preempt_requested():
			time.sleep(0.1)
		
		return 'ABORT'

def safetyWrap(task):
	def safety_outcome(outcome_map):
		if outcome_map['SAFETY'] == 'ABORT':
			return 'ABORT'
		elif outcome_map['SAFETY'] == 'RECOVERED':
			return 'RECOVERED'
		else:
			return outcome_map["TASK"]

	def safety_term(outcome_map):
		return True

	sm_wrapper = smach.Concurrence(list(task.get_registered_outcomes()) + ['ABORT', 'RECOVERED'],
									default_outcome='ABORT',
									outcome_cb=safety_outcome,
									child_termination_cb=safety_term)

	with sm_wrapper:
		smach.Concurrence.add("SAFETY", SafetyState())
		smach.Concurrence.add("TASK", task)

	return sm_wrapper
	

def main():
	rospy.init_node('hippo_sm')
	rospy.loginfo("State Machine has started.")

	mission_sm = smach.StateMachine(outcomes=['DONE', 'ABORT'])

	with mission_sm:

		smach.StateMachine.add('START', safetyWrap(StartState()), transitions={'GO':'GATE', 'ABORT':'ABORT', 'RECOVERED': 'START'})

		task_gate = safetyWrap(gate.makeTask())
		task_pole = safetyWrap(pole.makeTask())

		smach.StateMachine.add('GATE', task_gate, transitions={'DONE':'POLE', 'ABORT':'ABORT', 'RECOVERED':'GATE'})
		smach.StateMachine.add('POLE', task_pole, transitions={'DONE':'DONE','ABORT':'ABORT', 'RECOVERED':'POLE'})

	sis = smach_ros.IntrospectionServer("sm_server", mission_sm, "/MISSION")
	sis.start()
	result = mission_sm.execute()
	rospy.loginfo(result)
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
