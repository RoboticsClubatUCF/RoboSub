#! /usr/bin/env python
import rospy
import actionlib

import numpy as np
from numpy.linalg import norm

from scipy.integrate import odeint
#import quaternion #numpy quaternion lib from https://github.com/martinling/numpy_quaternion.git

import sub_trajectory.msg

def rosToArray(msg):
    return np.array([getattr(msg, key) for key in ["x", "y", "z", "w"] if hasattr(msg, key)]) #List comprehension (how ironic) for getting a vector from a message

#def rosToQuat(msg):
#    if hasattr(msg, "w") and hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):
#        return np.quaternion(getattr(msg, "w"), getattr(msg, "x"), getattr(msg, "y"), getattr(msg, "z"))
        
def slerp(p0, p1, t):
        omega = arccos(dot(p0/norm(p0), p1/norm(p1)))
        so = sin(omega)
        return sin((1.0-t)*omega) / so * p0 + sin(t*omega)/so * p1
        

class MovementServer:
    def __init__(self):
        rospy.loginfo("Movement server spooling up")
        self.server = actionlib.SimpleActionServer('move_vehicle', sub_trajectory.msg.GoToPoseAction, self.execute, False)
        
        self.ThrustControllerClient = actionlib.SimpleActionClient('thruster_waypoint', ExecuteWaypointAction)
        rospy.loginfo("Movement server waiting for thruster server")
        self.ThrustControllerClient.wait_for_server()
        
        self.server.start()
        
        #TODO: Set these values more(?) properly
        #TODO: Make these params or something
        self.thrust = np.array([35.0, 35.0, 35.0]) #Newtons, ignore asymetrical thrust limits
        
        self.vehicleMass = 32.0 #kilograms
        
        waterDensity = 1000 #kg/m^3 at 20C
        vehicleArea = np.array([0.1297, 0.2065, 0.2065]) #Front cross-section (make this a vector?)
        vehicleCd = 0.7 #S.W.A.G. TODO: make this a real number (also a vector? a function?)
        
        self.dragConstant = (waterDensity * vehicleArea * vehicleCd) / 2 #All the stuff that doesnt change in one number
        
        rospy.loginfo("Movement server ready")
        
    def motion_func(time, state, thrust, dragConstant, mass):
        f = zeros(2)
        f[0] = state[1] #velocity
        f[1] = (thrust - dragConstant * f[0]**2)/mass #acceleration
        
        return f
        
    def execute(self, goal):
        self.targetPosition = rosToArray(goal.targetPose.position)
        self.targetOrientation = rosToArray(goal.targetPose.orientation)
        
        self.startPosition = rosToArray(goal.startPose.position)
        self.startOrientation = rosToArray(goal.startPose.orientation)
        
        #Start by making waypoints with pose and velocity?
        #Doing straight line movements between waypoints for simplicity
        
        translationVector = self.targetPosition - self.startPosition
        directionVector = translationVector/np.linalg.norm(translationVector) #Direction we're travelling in
        
        maxThrust = np.dot(self.thrust, directionVector) #Figure out what our thrust output is
        thrustVector = self.thrust * maxThrust #Thrust vector for translation
        
        #TODO: better search for good acceleration stop times to avoid a negative cruise time
        #TODO: make this consider initial velocities and multiple waypoints (or switch to some actual dynamics library)
        #TODO: make this consider rotating and translating at the same time
        #TODO: make this consider different drag based on varying orientations
        y0 = [0.0, 0.0] #Initial conditions for acceleration predictions, 0 displacement 0 velocity (bad I know but hashtag 7 days)
        t1 = np.linspace(0, 3, 101) #Simulation window and timestep for acceleration, 3 seconds, 101 samples

        accelSol = odeint(motion_func, y0, t1, args=(maxThrust, self.dragConstant, self.vehicleMass)) #Simulate acceleration

        accelIdx = 1 #index to the solution arrays for when we're done accelerating

        for idx, val in enumerate(accelSol[:, 1]): #Find maximum acceleration velocity/time
            if val > max(accelSol[:, 1])*0.99 or accelSol[idx, 0] > distance/1.8: #Done accelerating if we're at full(ish) speed or dont have enough distance left(ish) to stop
                accelIdx = idx
                break
        
        accelTime = t1[accelIdx] #Store the time and distance taken to accelerate
        accelDist = accelSol[accelIdx, 0]
        accelVelocity = accelSol[accelIdx, 1] #Store final velocity after accelerating

        y0 = [0.0, accelVelocity] #Decelerate starting from max velocity
        t2 = np.linspace(0, accelTime, 101) #shouldnt take longer to decelerate than it took to accelerate

        decelSol = odeint(motion_func, y0, t2, args=(-maxThrust, self.dragConstant, self.vehicleMass)) #Decelerate at max thrust
        decelIdx = 1 #Index to solution arrays for decelerating

        for idx, val in enumerate(decelSol[:, 1]): #Find deceleration time
            if val < 0.02:
                decelIdx = idx
                break

        decelTime = t2[decelIdx]
        decelDist = decelSol[decelIdx, 0]

        cruiseDist = distance - (accelDist+decelDist)
        cruiseTime = cruiseDist/accelVelocity
        
        acceleratePositionTarget = self.startPosition + directionVector * accelDist
        accelerateVelocityTarget = accelVelocity * directionVector
        
        cruisePositionTarget = acceleratePositionTarget + directionVector * max(cruiseDist, 0) #Make sure we dont try to cruise backwards
        cruiseVelocityTarget = accelerateVelocityTarget #Cruise just tries to maintain our max velocity
        
        accelerate = sub_trajectory.msg.ExecuteWaypointGoal() #Waypoint action for the acceleration phase of the vehicle movement
        accelerate.header.frame = "map" #Or some other world frame, no fucking idea at this time of night
        accelerate.stationKeeping = False
        accelerate.targetPose.position = geometry_msgs.msg.Point(*acceleratePositionTarget) 
        accelerate.targetPose.orientation = geometry_msgs.msg.Quaternion(*self.startOrientation) #maintain current orientation so the math is easier
        accelerate.targetTwist.linear = geometry_msgs.msg.Vector3(*accelerateVelocityTarget)
        accelerate.targetTwist.angular = geometry_msgs.msg.Vector3(0,0,0) #Keep 0 orientation velocity
        
        cruise = sub_trajectory.msg.ExecuteWaypointGoal() #waypoint action for cruise phase of vehicle movement
        cruise.header.frame = "map"
        cruise.stationKeeping = False
        cruise.targetPose.position = geometry_msgs.msg.Point(*cruisePositionTarget)
        cruise.targetPose.orientation = geometry_msgs.msg.Quaternion(*self.startOrientation)
        cruise.targetTwist.linear = geometry_msgs.msg.Vector3(*cruiseVelocityTarget)
        cruise.targetTwist.angular = geometry_msgs.msg.Vector3(0,0,0)
        
        decelerate = sub_trajectory.msg.ExecuteWaypointGoal() #Waypoint for deceleration phase
        decelerate.header.frame = "map"
        decelerate.stationKeeping = False
        decelerate.targetPose.position = geometry_msgs.msg.Point(*self.targetPosition)
        decelerate.targetPose.orientation = geometry_msgs.msg.Quaternion(*self.startOrientation)
        decelerate.targetTwist.linear = geometry_msgs.msg.Vector3(0,0,0)
        decelerate.targetTwist.angular = geometry_msgs.msg.Vector3(0,0,0)
        
        rotate = sub_trajectory.msg.ExecuteWaypointGoal() #Waypoint for rotation phase
        rotate.header.frame = "map"
        rotate.stationKeeping = True
        rotate.targetPose.position = geometry_msgs.msg.Point(*self.targetPosition)
        rotate.targetPose.orientation = geometry_msgs.msg.Quaternion(*self.targetOrientation)
        rotate.targetTwist.linear = geometry_msgs.msg.Vector3(0,0,0)
        rotate.targetTwist.angular = geometry_msgs.msg.Vector3(0,0,0)
        
        waypoints = [accelerate, cruise, decelerate, rotate] #Put the 4 movement phases in a list
        
        #TODO: catch errors and abort as appropriate
        #TODO: add feedback/error result stuff
        for waypoint in waypoints: #Execute the 4 part movement plan
            self.ThrusterControllerClient.send_goal(waypoint)
            while self.ThrusterControllerClient.get_state() is in ["ACTIVE", "PENDING"]:
                if self.server.is_preempt_requested():
                    self.ThrusterControllerClient.cancel_all_goals()
                    self.server.set_preempted(result=sub_trajectory.msg.GoToPoseResult(Ok=True))
                    return
                
                
        self.server.set_succeeded(result=sub_trajectory.msg.GoToPoseResult(Ok=True))
        
if __name__ == '__main__':
    rospy.init_node('movement_server')
    server = MovementServer()
    rospy.spin()        
