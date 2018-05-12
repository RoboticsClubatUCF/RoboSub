#!/user/bin/env python
import rospy
import smach
import actionlib
import actionlib_msgs.msg
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult, VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult

class locate(smach.State):
	def __init__(self):
        	smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
        	self.client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
        	self.client.wait_for_server()

	def execute(self, userdate):
        	rospy.loginfo("Locating the pole.")
        	start = rospy.Time(0)

        	goal = TrackObjectGoal()
        	goal.objectType = goal.pole
		goal.servoing = False
        	self.client.send_goal(goal)

		self.client.wait_for_result()
		result = self.client.get_result()
		#rospy.loginfo(self.feedback.found)
		if result.found:
			return 'success'

		else:
			return 'failure'


class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
        	self.client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        	self.client.wait_for_server()

        def execute(self, userdata):
        	rospy.loginfo("Aligning with the pole.")
            	start = rospy.Time(0)

            	goal = VisualServoGoal()
            	goal.servotask = goal.align
            	self.client.send_goal(goal)

            	self.client.wait_for_result()
    		aligned = self.client.get_result()

            	if aligned.aligned:
                	return 'success'

            	else:
                	return 'failure'


class drift(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
        	self.client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        	self.client.wait_for_server()

    	def execute(self, userdata):
    		rospy.loginfo("Drifting")
        	start = rospy.Time(0)

        	goal = TrackObjectGoal()
        	goal.servotask = goal.drift
        	self.client.send_goal(pole)

        	self.client.wait_for_result()
        	drifted = self.client.get_result()

        	if drifted.aligned:
            		rospy.loginfo("Drifting complete!")
            		return 'sucess'

        	else:
            		return 'failure'

