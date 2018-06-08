#!/user/bin/env python
import rospy
import tf
import actionlib
import actionlib_msgs.msg
import vision_manager.msg
import visual_servo.msg
import visual_servo as vs
from geometry_msgs.msg import Wrench

class locate(smach.State):
	def __init__(self):
        smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
        self.vision_client = actionlib.SimpleActionClient('track_object')
        self.vision_client.wait_for_server()

	def execute(self, userdate):
        rospy.loginfo("Locating the pole.")
        start = rospy.Time(0)
        goal = vision_manager.msg.TrackObjectGoal()
        goal.objectType = goal.pole
        self.vision_client.send_goal(goal)

        vision_client.wait_for_result()
        found = vision_client.get_result()

		if found:
            rospy.loginfo("Pole located.")
            return 'success'

		if not found:
            return 'failure'

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure']):
        self.servo_client = actionlib.SimpleActionClient('visual_servo')
        self.servo_client.wait_for_server()

        def execute(self, userdata):
            rospy.loginfo("Aligning with the pole.")
            start = rospy.Time(0)
		
            goal = vision_manager.msg.TrackObjectGoal()
        	goal.objectType = goal.pole
            self.vision_client.send_goal(goal)

            goal = visual_servo.msg.TrackObjectGoal()
            goal.servotask = goal.align
            self.client.send_goal(servoGoal)

            client.wait_for_result()
    		aligned = client.get_result():
	        
            if aligned:
                return 'success'
                       
            else:
                return 'failure'


class drift(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
        self.client = actionlib.SimpleActionClient('visual_servo')
        self.client.wait_for_server()                

    def execute(self, userdata):
        rospy.loginfo("Drifting")
        start = rospy.Time(0)

        goal = visual_servo.msg.TrackObjectGoal()
        goal.servotask = goal.drift
        self.client.send_goal(pole)

        client.wait_for_result()
        drifted = client.get_result():
        
        if drifted: 
            rospy.loginfo("Drifting complete!")
            return 'sucess'
    
        else:
            return failure
    