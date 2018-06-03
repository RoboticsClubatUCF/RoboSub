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
                self.listener = tf.TransformListener()

	def execute(self, userdate):
                rospy.loginfo("Locating the pole.")
                start = rospy.Time(0)
                goal = vision_manager.msg.TrackObjectGoal()
                goal.objectType = goal.pole
                self.vision_client.send_goal(goal)

               	found = vision_manager.msg.TrackObjectResult()

		if not found:
	                found = vision_manager.msg.TrackObjectResult()

                if found:
                        rospy.loginfo("Pole located.")
                        return 'success'

		if not found:
                        return 'failure'

class align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','success', 'failure'])
                self.servo_client = actionlib.SimpleActionClient('visual_servo')
                self.servo_client.wait_for_server()

        def execute(self, userdata):
                rospy.loginfo("Aligning with the pole.")
                start = rospy.Time(0)
		goal = vision_manager.msg.TrackObjectGoal()
		goal.objectType = goal.pole
		self.vision_client.send_goal(goal)

		while True:
                        if visual_servo.msg.TrackObjectResult().aligned:
	                        return 'success'
                        else:
                                pass
                return 'failure'


class drift(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
