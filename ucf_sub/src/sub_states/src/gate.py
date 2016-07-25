#!/usr/bin/env python

class locate(smach.State):
	"""
	Locates the gate. It should be viewable through the front cameras on startup,
	but in case it isn't it should look around to find it. 

	Look a little left and a little right? Full rotation? Up and down?
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
		attempts = 5

	def execute(self, userdata):
		rospy.loginfo("Locating the gate.")

		# found = Call vision to see if gate is in view of front camereas.
		
		if found:
			rospy.loginfo("Gate located.")
			return 'success'
		if not found:
			rospy.loginfo("Gate not located. Attempting to locate.")
			for num in range(0, attempts):
				rospy.loginfo("Location attempt {}.".format(num))
				if self.found:
					rospy.loginfo("Gate located on attempt {}.".format(num))
					return 'success'
				if self.preemption():
					return 'preempted'
			return 'failure'

class align(smach.State):
	"""
	Aligns the submarine with the gate so that passing through is (hopefully) a straight shot.
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])

	def execute(self, userdata):
		rospy.loginfo("Aligning the gate.")


class through(smach.State):
	"""
	Submarine goes straight forward through the gate.
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])

	def execute(self, userdata):
		rospy.loginfo("Attempting to touch the {} buoy.".format(buoyColor))
		
		# move forward
			rospy.loginfo("Moving forward to touch the {} buoy.".format(buoyColor))
			#moce forward

		# successful
			rospy.loginfo("The {} buoy was successfully touched.".format(buoyColor))
			# reverse
