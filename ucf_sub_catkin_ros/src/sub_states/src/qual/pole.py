#!/user/bin/env python
import rospy
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus
from sub_vision.msg import TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult, VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
import tf

class locate(smach.State):
    def __init__(self):
            smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
            self.client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
            self.client.wait_for_server()
            self.firstIMUCall = True
            self.imuSubscriber = rospy.Subscriber("/imu/data", Imu, self.initIMU)
            self.thresh = tf.transformations.quaternion_from_euler(0,0,15)
            self.orientationX = None
            self.orientationY = None
            self.orientationZ = None
            self.com = Wrench()

    def execute(self, userdate):
        rospy.loginfo("Locating the pole.")
        start = rospy.Time(0)

        goal = TrackObjectGoal()
        goal.objectType = goal.pole
        goal.servoing = False
        self.client.send_goal(goal)

        while self.client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING] and not self.preempt_requested():
            time.sleep(0.1)

        if self.preempt_requested():
            return 'preempted'

        result = self.client.get_result()
        #rospy.loginfo(self.feedback.found)
        if result.found:
            return 'success'

        else:
            if (self.orientationZ < self.thresh and self.orientationZ > self.startZ) or self.orientationZ == None:
                self.com.force.x = 0
                self.com.force.y = 0
                self.com.force.z = 0
                self.com.torque.x = 0
                self.com.torque.y = 0
                self.com.torque.z = 1

            else:

                return 'failure'

    
    def initIMU(self, msg):
        if self.firstIMUCall:
            self.startX = msg.orientation.x
            self.startY = msg.orientation.y
            self.startZ = msg.orientation.z
            self.firstIMUCall = False
            self.thresh = tf.transformations.quaternion_multiply(self.startZ, self.thresh)
            return 

        self.orientationX = msg.orientation.x
        self.orientationY = msg.orientation.y
        self.orientationZ = msg.orientation.z



class align(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'success', 'failure'])
        self.client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo("Aligning with the pole.")
        start = rospy.Time(0)

        goal = VisualServoGoal()
        goal.servotask = goal.align
        self.client.send_goal(goal)

        while self.client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING] and not self.preempt_requested():
            time.sleep(0.1)

        if self.preempt_requested():
            return 'preempted'
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

        goal = VisualServoGoal()
        goal.servotask = goal.drift
        self.client.send_goal(goal)

        while self.client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING] and not self.preempt_requested():
            time.sleep(0.1)

        if self.preempt_requested():
            return 'preempted'

        drifted = self.client.get_result()

        if drifted.aligned:
            rospy.loginfo("Drifting complete!")
            return 'sucess'

        else:
            return 'failure'

def makeTask():
    task = smach.StateMachine(outcomes=['DONE'])
    with task:
        smach.StateMachine.add('LOCATE', locate(),
                        transitions={'preempted':'DONE',
                            'success': 'ALIGN',
                            'failure': 'LOCATE'})

        smach.StateMachine.add('ALIGN', align(),
                        transitions={'preempted':'DONE',
                            'success': 'DRIFT',
                            'failure': 'LOCATE'})

        smach.StateMachine.add('DRIFT', drift(),
                        transitions={'preempted':'DONE',
                            'success': 'DONE',
                            'failure': 'LOCATE'})
    return task