#! /usr/bin/env python
import rospy
import actionlib
import tf
from tf import transformations

import numpy as np
from numpy.linalg import norm
import math

import sub_trajectory.msg
import geometry_msgs.msg
import nav_msgs.msg

def slerp(p0, p1, t):
        omega = arccos(dot(p0/norm(p0), p1/norm(p1)))
        so = sin(omega)
        return sin((1.0-t)*omega) / so * p0 + sin(t*omega)/so * p1
        
def rosToArray(msg):
    return np.array([getattr(msg, key) for key in ["x", "y", "z", "w"] if hasattr(msg, key)]) #List comprehension (how ironic) for getting a vector from a message
    
def quatErrorToAxis(a, b): #adapted from parts of CUAUV's code
    a = np.array([-a[0], -a[1], -a[2], a[3]])
    c = transformations.quaternion_multiply(b, a)
    axis = c[0:3]/math.sin(math.acos(c[3]))
    angle = math.acos(c[3])*2
    
    if angle > math.pi:
        axis = -axis
        angle = 2*math.pi - angle
        
    return axis*angle
    
class ThrusterServer:
    def __init__(self):
        rospy.loginfo("Thruster server spooling up")
        self.server = actionlib.SimpleActionServer('drive_thrusters', sub_trajectory.msg.ExecuteWaypoint, self.execute, False)
        self.server.start()
        
        self.tfListener = tf.TransformListener()
        
        self.thrusterPublisher = rospy.Publisher('thrusters/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        
        self.odomListener = rospy.Subscriber("/odom", nav_msgs.msg.Odometry, self.odomCallback)
        self.odom = None
        
        #sick gains bruh, TODO: tune these
        self.gains = {"kp_linear" : 1.0, "kd_linear": 1.0, "kp_angular": 0.5, "kd_angular" : 0.5}
        
    def odomCallback(msg):
        self.odom = msg
        
    def execute(self, goal): #Largely inspired by UF's base_controller.py but as an actionserver and only one waypoint at a time
        self.complete = False
        rate = rospy.rate(50)
        
        #TODO: make this consider which tf frame the goal is in
        #UF doesnt seem to worry about this being in a wierd frame so why should we
        targetPosition = rosToArray(goal.targetPose.position)
        targetOrientation = rosToArray(goal.targetPose.orientation)
        
        targetLinearVelocity = rosToArray(goal.targetTwist.linear)
        targetAngularVelocity = rosToArray(goal.targetTwist.angular)
        
        while not complete:
        
            #TODO: finish this
            if self.server.is_new_goal_available():
                pass
            
            if self.odom is None: #Cant really drive the robot if we dont have odometry data
                continue
            
            if self.server.is_preempt_requested():
                thrust_cmd = geometery_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3([0,0,0])
                    angular=geometry_msgs.msg.Vector3([0,0,0])
                )
                self.thrusterPublisher.publish(thrust_cmd)
                self.server.set_preempted(result=sub_trajectory.msg.ExecuteWaypointResult(Ok=True))
                return
                
            #shut down if we havent had odometry in the last 3 seconds
            #TODO: Finish this
            #TODO: Make this use a param or something
            if rospy.Time.now() - self.odom.header.stamp > 3:
                pass
                
            #TODO: consider the tf frame of odom
            positionError = rosToArray(self.odom.pose.pose.position) - targetPosition
            orientationError = quatErrorToAxis(rosToArray(self.odom.pose.pose.orientation), targetOrientation)
            
            linearVelocityError = rosToArray(self.odom.twist.twist.linear) - targetLinearVelocity
            angularVelocityError = rosToArray(self.odom.twist.twist.angular) - targetAngularVelocity
            
            if np.linalg.norm(positionError) < 0.05 and np.linalg.norm(orientationError) < 0.1 
                and np.linalg.norm(linearVelocityError) < 0.05 and np.linalg.norm(angularVelocityError) < 0.05 
                and not goal.stationKeeping:
                
                complete=True
                thrust_cmd = geometery_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3([0,0,0])
                    angular=geometry_msgs.msg.Vector3([0,0,0])
                )
                self.thrusterPublisher.publish(thrust_cmd)
                
                continue
            world_linear = self.gains["kp_linear"] * positionError + self.gains["kd_linear"] * linearVelocityError
            world_angular = self.gains["kp_angular"] * orientationError + self.gains["kd_angular"] * angularVelocityError
            
            worldToBody = transformations.quaternion_matrix(rosToArray(self.odom.pose.pose.orientation))[:3,:3]
            
            body_linear = worldToBody.dot(world_linear)
            body_angular = worldToBody.dot(world_angular)
            
            thrust_cmd = geometery_msgs.msg.Twist(
                linear=geometry_msgs.msg.Vector3(*body_linear)
                angular=geometry_msgs.msg.Vector3(*body_angular)
                )
                
           self.thrusterPublisher.publish(thrust_cmd)
           rate.sleep()
       set_succeeded(result=sub_trajectory.msg.ExecuteWaypointResult(Ok=True))
       
if __name__ == '__main__':
    rospy.init_node('thruster_server')
    server = ThrusterServer()
    rospy.spin()  
