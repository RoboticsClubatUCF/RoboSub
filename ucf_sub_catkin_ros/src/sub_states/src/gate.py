#!/user/bin/env python
import smach
import rospy
import time
import actionlib
from actionlib_msgs.msg import GoalStatus
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult, VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult
from geometry_msgs.msg import  

class through(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])


	def execute(self, userdata):
		self.stabilityPub = rospy.Publisher("/thrusters/depthMode", StabilityMode, queue_size=1)
		self.stabilityMsg = StabilityMode()

		self.orientationPub = rospy.Publisher("/thrusters/orientationMode", StabilityMode, queue_size=1)
		self.orientationMsg = StabilityMode()

		self.stabilityMsg.target.z=1
		self.stabilityMsg.mode = StabilityMode.position
		self.stabilityPub.publish(self.stabilityMsg)

		self.orientationMsg.target.w=float("nan")
		self.orientationMsg.mode = StabilityMode.position
		self.orientationMsg.yawEnabled = True
		self.orientationPub.publish(self.stabilityMsg)
		
		self.thruster_pub = rospy.Publisher('/autonomyWrench', WrenchStamped, queue_size=1)
		self.autonomyMsg = WrenchStamped()

		self.autonomyMsg.x = 3
		self.thruster_pub.publish(self.autonomyMsg)

		t0 = time.time()

		while time.time()-t0 > 10 and not self.preempt_requested():
			time.sleep(0.01)

		self.autonomyMsg.x = 0.5
		self.thruster_pub.publish(self.autonomyMsg)

		if self.preempt_requested():
			self.service_preempted()
			return 'preempted'

		return 'success'

def makeTask():
	task = smach.StateMachine(outcomes=['DONE'])
	with task:
		smach.StateMachine.add('THROUGH', through(),
						transitions={'preempted':'DONE',
							'success': 'DONE',
							'failure':'through'})
	
	return task