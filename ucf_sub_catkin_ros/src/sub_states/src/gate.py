#!/user/bin/env python
import smach
import rospy
import time
import actionlib
from actionlib_msgs.msg import GoalStatus
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult
from sub_vision.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult
from sub_vision.msg import feedback
from geometry_msgs.msg import  WrenchStamped

from sub_trajectory.msg import StabilityMode

class through(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
        self.stabilityPub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
        self.orientationPub = rospy.Publisher("/thrusters/angleMode", StabilityMode, queue_size=1)
        self.thruster_pub = rospy.Publisher('/autonomyWrench', WrenchStamped, queue_size=1)
        self.vision_client = actionlib.SimpleActionClient('track_object', TrackObjectAction)

        self.autonomyMsg = WrenchStamped()

        self.coeff = 1
        self.t0 = 0
        self.vision_feedback = rospy.Subscriber('/gate_feedback', feedback, self.feedbackCb)

    def feedbackCb(self, msg):
        self.autonomyMsg.wrench.force.y = (msg.center[0]-msg.size[0]/2) * self.coeff / msg.size[0]
        rospy.loginfo_throttle(5, self.autonomyMsg.wrench.force.y)
        self.t0 = time.time()
        
    def execute(self, userdata):

        self.stabilityMsg = StabilityMode()
        self.orientationMsg = StabilityMode()

        self.stabilityMsg.target.z=2
        self.stabilityMsg.mode = StabilityMode.position
        self.stabilityPub.publish(self.stabilityMsg)

        self.orientationMsg.target.w=float("nan")
        self.orientationMsg.mode = StabilityMode.position
        self.orientationMsg.yawEnabled = True
        self.orientationPub.publish(self.orientationMsg)

        goal = TrackObjectGoal()
        goal.objectType = goal.startGate
        goal.servoing = True
        self.vision_client.send_goal(goal)

        self.autonomyMsg.wrench.force.x = 2
        self.thruster_pub.publish(self.autonomyMsg)

        self.t0 = time.time()

        while ((time.time()-self.t0) < 60) and not self.preempt_requested():
            time.sleep(0.01)
            self.autonomyMsg.header.stamp = rospy.Time.now()
            self.thruster_pub.publish(self.autonomyMsg)

        self.autonomyMsg.wrench.force.x = 0.0
        self.thruster_pub.publish(self.autonomyMsg)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'success'

def makeTask():
    task = smach.StateMachine(outcomes=['DONE'])
    with task:
        smach.StateMachine.add('THROUGH', through(),
                        transitions={'preempted':'DONE',
                            'success': 'DONE',
                            'failure':'THROUGH'})
    
    return task
