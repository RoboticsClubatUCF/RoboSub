#!/usr/bin/env python
import rospy
import tf
import actionlib
import actionlib_msgs.msg
import vision_manager.msg
import visual_servo as vs
import trajectory_planner.msg
from geometry_msgs.msg import Wrench

class locate(smach.State):
	"""
	Locates the gate. It should be viewable through the front cameras on startup,
	but in case it isn't it should look around to find it. 

	Look a little left and a little right? Full rotation? Up and down?
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
                self.vision_client = actionlib.SimpleActionClient('track_object')
                self.vision_client.wait_for_server()
                self.movement_client = actionlib.SimpleActionClient('movement_server')
                self.movement_client.wait_for_server()
                self.listener = tf.TransformListener()
                self.rel_pose = None
	def execute(self, userdata):
		rospy.loginfo("Locating the gate.")
                start = rospy.Time(0)
		# found = Call vision to see if gate is in view of front camereas.
                goal = vision_manager.msg.TrackObjectGoal()
                goal.objectType = goal.startGate
		while self.rel_pose == None:
                        self.vision_client.send_goal(goal, feedback_cb = feedback_cb)
                        if self.rel_pose != None:
                                found = true
                                break
                        if rospy.Time(0) - start == 90:
                                found = false
                                break
                        move_goal = trajectory_planner.msg.GoToPoseGoal()
                        t = self.listener.getLatestCommonTime('/base_link','/map')
                        position, rotation = self.listener.lookupTransform('/base_link','/map', t)
                        move_goal.startPose.position.x = position.x
                        move_goal.startPose.position.y = position.y
                        move_goal.startPose.position.z = position.z
                        move_goal.startPose.orientation.w = rotation.w
                        move_goal.startPose.orientation.x = rotation.x
                        move_goal.startPose.orientation.y = rotation.y
                        move_goal.startPose.orientation.z = rotation.z
                        quaternion = tf.transformations.quaternion_from_euler(0, 0, 90)
                        move_goal.targetPose.position.x = position.x
                        move_goal.targetPose.position.y = position.y
                        move_goal.targetPose.position.z = position.z
                        move_goal.targetPose.orientation.x = quaternion[0]
                        move_goal.targetPose.orientation.y = quaternion[1]
                        move_goal.targetPose.orientation.z = quaternion[2]
                        move_goal.targetPose.orientation.w = quaternion[3]
                        self.movement_client.send_goal(move_goal)
                if found:
			rospy.loginfo("Gate located.")
			return 'success'
		if not found:
			return 'failure'


        def feedback_cb(self,feedback):
                if feedback is not None:
                        try:
                                (trans,rot) = self.listener.lookupTransform('/camera_frame','/base_link', rospy.Time(0)) #TODO: fix frame names
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue
                                if feedback.fixType == feedback.pose or feedback.fixType == feedback.position:
                                        self.rel_pose.translation.x = feedback.targetPose.translation.x - trans.x
                                        self.rel_pose.translation.y = feedback.targetPose.translation.y - trans.y
                                        self.rel_pose.translation.z = feedback.targetPose.translation.z - trans.z
                                if feedback.fixType == feedback.pose or feedback.fixType == feedback.direction:
                                        self.rel_pose.rotation.w = feedback.targetPose.rotation.w
                                        self.rel_pose.rotation.x = feedback.targetPose.rotation.x
                                        self.rel_pose.rotation.y = feedback.targetPose.rotation.y
                                        self.rel_pose.rotation.z = feedback.targetPose.rotation.z

class align(smach.State):
	"""
	Aligns the submarine with the gate so that passing through is (hopefully) a straight shot.
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])

	def execute(self, userdata):
		rospy.loginfo("Aligning the gate.")
		vs.main() #Call visual servoing

class through(smach.State):
	"""
	Submarine goes straight forward through the gate.
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
		self.message = Wrench()

	#Set to 1 in the +x direction for now
	def execute(self, userdata):
		message.force.x = 1
		message.force.y = 0
		message.force.z = 0
		message.torque.x = 0
		message.torque.y = 0
		message.torque.z = 0
