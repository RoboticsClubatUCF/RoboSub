#!/user/bin/env python
import rospy
import smach
import actionlib
import actionlib_msgs.msg
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult, VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult

class locate(smach.State):
	def __init__(self):
        	smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
        	self.vision_client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
        	self.vision_client.wait_for_server()

	def execute(self, userdate):
        	rospy.loginfo("Locating the pole.")
        	start = rospy.Time(0)
        	goal = TrackObjectGoal()
        	goal.objectType = goal.pole
        	self.vision_client.send_goal(goal)

        	vision_client.wait_for_result()
        	found = vision_client.get_result()

		if found.found:
            		rospy.loginfo("Pole located.")
            		return 'success'

		else:
            		return 'failure'

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
        	self.servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        	self.servo_client.wait_for_server()

        def execute(self, userdata):
        	rospy.loginfo("Aligning with the pole.")
            	start = rospy.Time(0)
		
            	goal = TrackObjectGoal()
        	goal.objectType = goal.pole
            	self.vision_client.send_goal(goal)

            	goal = TrackObjectGoal()
            	goal.servotask = goal.align
            	self.client.send_goal(servoGoal)

            	client.wait_for_result()
    		aligned = client.get_result()
	        
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

        	client.wait_for_result()
        	drifted = client.get_result()
        
        	if drifted.aligned: 
            		rospy.loginfo("Drifting complete!")
            		return 'sucess'
    
        	else:
            		return failure
    
