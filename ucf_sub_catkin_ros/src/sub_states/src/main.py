#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool
from sub_trajectory.msg import StabilityMode
import smach, smach_ros

import gate
import path
import dice
import slots

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

class StopState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['DONE', 'RESTART'],
							input_keys=['depth'])
		self.stabilityPub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
		self.stabilityMsg = StabilityMode()
		self.waiting = True
		rospy.Subscriber("/start", Bool, self.button)

	def button(self, msg):
		self.waiting=False

	def execute(self, userdata):
		self.stabilityMsg.target.z=userdata.depth
		self.stabilityMsg.mode = StabilityMode.position
		self.stabilityPub.publish(self.stabilityMsg)

		while self.waiting and not self.preempt_requested():
			time.sleep(0.1)

		return 'RESTART'

class SafetyState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['ABORT', 'RECOVERED', 'PREEMPTED'])
		self.leak = False
		rospy.Subscriber("/leak", Bool, self.leakCb)

	def leakCb(self, msg):
		self.leak = msg.data
	
	def execute(self, userdata):
		while not self.leak and not self.preempt_requested():
			time.sleep(0.1)
		
		if self.preempt_requested():
			return 'PREEMPTED'
		else:
			return 'ABORT'

def safetyWrap(task):
	def safety_outcome(outcome_map):
		if outcome_map['SAFETY'] in ['ABORT', 'RECOVERED']:
			return outcome_map['SAFETY']
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

	mission_sm = smach.StateMachine(outcomes=['DONE'])
	mission_sm.userdata.abort_depth = -10
	mission_sm.userdata.stop_depth = 1

	with mission_sm:

		smach.StateMachine.add('START', safetyWrap(StartState()), transitions={'GO':'GATE', 'ABORT':'ABORT', 'RECOVERED': 'START'})

		task_gate = safetyWrap(gate.makeTask())
		task_path1 = safetyWrap(path.makeTask())
		task_dice = safetyWrap(dice.makeTask())
		task_path2 = safetyWrap(path.makeTask())
		task_slots = safetyWrap(slots.makeTask())

		smach.StateMachine.add('GATE', task_gate, transitions={'DONE':'PATH', 'ABORT':'ABORT', 'RECOVERED':'GATE'})
		smach.StateMachine.add('PATH1', task_path1, transitions={'DONE':'DICE', 'ABORT':'ABORT', 'RECOVERED':'PATH1'})
		smach.StateMachine.add('DICE', task_dice, transitions={'DONE':'PATH2', 'ABORT':'ABORT', 'RECOVERED':'DICE'})
		smach.StateMachine.add('PATH2', task_path2, transitions={'DONE':'SLOTS', 'ABORT':'ABORT', 'RECOVERED':'PATH2'})
		smach.StateMachine.add('SLOTS', task_slots, transitions={'DONE':'STOP', 'ABORT':'ABORT', 'RECOVERED':'SLOTS'})

		smach.StateMachine.add('STOP', safetyWrap(StopState()), transitions={'RESTART':'START', 'DONE':'DONE'}, remapping={'depth':'stop_depth'})
		smach.StateMachine.add('ABORT', safetyWrap(StopState()), transitions={'RESTART':'START', 'DONE':'DONE'}, remapping={'depth':'abort_depth'})

	sis = smach_ros.IntrospectionServer("sm_server", mission_sm, "/MISSION")
	sis.start()
	result = mission_sm.execute()
	rospy.loginfo(result)
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()
