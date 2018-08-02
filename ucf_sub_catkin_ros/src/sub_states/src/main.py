#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool
from sub_trajectory.msg import StabilityMode
import smach, smach_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import WrenchStamped
import gate
import path
import dice
import slots

class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['GO', 'PREEMPTED'])
        self.waiting = True
        rospy.Subscriber("/start", Bool, self.go)
        self.depthPub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
        self.anglePub = rospy.Publisher("/thrusters/angleMode", StabilityMode, queue_size=1)

        self.stabilityMsg = StabilityMode()
        self.stabilityMsg.mode = StabilityMode.off

        self.thruster_pub = rospy.Publisher('/autonomyWrench', WrenchStamped, queue_size=1)
        self.autonomyMsg = WrenchStamped()

    def go(self, msg):
        self.waiting = False

    def execute(self, userdata):
        self.depthPub.publish(self.stabilityMsg)
        self.anglePub.publish(self.stabilityMsg)
        
        self.waiting = True
        while self.waiting and not self.preempt_requested():
            self.autonomyMsg.header.stamp = rospy.Time.now()
            self.thruster_pub.publish(self.autonomyMsg)
            time.sleep(0.02)

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        return 'GO'

class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DONE', 'RESTART', 'PREEMPTED'],
                            input_keys=['depth'])
        self.stabilityPub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
        self.stabilityMsg = StabilityMode()
        self.waiting = True
        rospy.Subscriber("/start", Bool, self.button)

        self.thruster_pub = rospy.Publisher('/autonomyWrench', WrenchStamped, queue_size=1)
        self.autonomyMsg = WrenchStamped()

    def button(self, msg):
        self.waiting=False

    def execute(self, userdata):
        self.waiting = True
        self.stabilityMsg.target.z=0.5
        self.stabilityMsg.mode = StabilityMode.position
        self.stabilityPub.publish(self.stabilityMsg)

        while self.waiting and not self.preempt_requested():
            time.sleep(0.02)
            self.autonomyMsg.header.stamp = rospy.Time.now()
            self.thruster_pub.publish(self.autonomyMsg)

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        return 'RESTART'

class SafetyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ABORT', 'RECOVERED', 'PREEMPTED', 'ESTOP'])
        self.critError = False
        self.estop = False
        rospy.Subscriber("/leak", Bool, self.leakCb)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.depthStatus)

    def leakCb(self, msg):
        if msg.data:
            self.critError = True

    def depthStatus(self, msg):
        self.estop = False
        thrusterErrors = 0
        for status in msg.status:
            if status.name == 'DepthDisconnect':
                if status.message == 'true':
                    self.critError = True
            elif 'Thruster_' in status.name and status.level != 0:
                thrusterErrors += 1
        if thrusterErrors > 7:
            self.estop = True
            rospy.logwarn("ESTOP PROBABLY ON")
        elif thrusterErrors > 1:
            rospy.logwarn("THRUSTERS PROBABLY BUSTED")
            self.critError = True

    def execute(self, userdata):
        while not self.critError and not self.estop and not self.preempt_requested():
            time.sleep(0.1)

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'
        elif self.estop:
            self.estop = False
            return 'ESTOP'
        else:
            self.critError = False
            return 'ABORT'

def safetyWrap(task):
    def safety_outcome(outcome_map):
        rospy.logerr(outcome_map)
        if outcome_map['SAFETY'] != 'PREEMPTED':
            return outcome_map['SAFETY']
        elif outcome_map['TASK'] is not None:
            return outcome_map['TASK']
        else:
            return 'ABORT'

    def safety_term(outcome_map):
        return True

    safetyState = SafetyState()
    outcomes = list(task.get_registered_outcomes()) + list(safetyState.get_registered_outcomes())
    outcomes[:] = [x for x in outcomes if x != 'PREEMPTED']

    sm_wrapper = smach.Concurrence(outcomes,
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

        smach.StateMachine.add('START', safetyWrap(StartState()), transitions={'GO':'GATE', 'ABORT':'ABORT', 'RECOVERED': 'START', 'ESTOP': 'START'})

        task_gate = safetyWrap(gate.makeTask())
        task_path1 = safetyWrap(path.makeTask())
        task_dice = safetyWrap(dice.makeTask())
        task_path2 = safetyWrap(path.makeTask())
        task_slots = safetyWrap(slots.makeTask())

        smach.StateMachine.add('GATE', task_gate, transitions={'DONE':'STOP', 'ABORT':'ABORT', 'RECOVERED':'GATE', 'ESTOP': 'START'})
        smach.StateMachine.add('PATH1', task_path1, transitions={'DONE':'DICE', 'ABORT':'ABORT', 'RECOVERED':'PATH1', 'ESTOP': 'START'})
        smach.StateMachine.add('DICE', task_dice, transitions={'DONE':'PATH2', 'ABORT':'ABORT', 'RECOVERED':'DICE', 'ESTOP': 'START'})
        smach.StateMachine.add('PATH2', task_path2, transitions={'DONE':'SLOTS', 'ABORT':'ABORT', 'RECOVERED':'PATH2', 'ESTOP': 'START'})
        smach.StateMachine.add('SLOTS', task_slots, transitions={'DONE':'STOP', 'ABORT':'ABORT', 'RECOVERED':'SLOTS', 'ESTOP': 'START'})

        smach.StateMachine.add('STOP', safetyWrap(StopState()), transitions={'RESTART':'START', 'DONE':'DONE', 'ABORT':'ABORT', 'RECOVERED':'STOP', 'ESTOP':'START'})
        smach.StateMachine.add('ABORT', StopState(), transitions={'RESTART':'START', 'DONE':'DONE', 'PREEMPTED':'DONE'}, remapping={'depth':'abort_depth'})

    sis = smach_ros.IntrospectionServer("sm_server", mission_sm, "/MISSION")
    sis.start()
    result = mission_sm.execute()
    rospy.loginfo(result)
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
