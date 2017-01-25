#! /usr/bin/env python
import rospy

from sub_trajectory.msg import ThrusterStatus
from geometry_msgs.msg import Wrench

import numpy as np

def rosToArray(msg):
    return np.array([getattr(msg, key) for key in ["x", "y", "z", "w"] if hasattr(msg, key)]) #List comprehension (how ironic) for getting a vector from a message

def rosToWrench(msg):
    return np.hstack((rosToArray(msg.force), rosToArray(msg.torque)))
    
class VectorController:

    def __init__(self): #Get parameters from rosparam/URDF/TF, get thruster objects ready
        self.thrusterStatuses = dict()
        self.thrusterData = {int(k):v for k,v in rospy.get_param("/thrusters").items()}
        
        self.thrustToWrench = np.zeros((6, len(self.thrusterData)))
        self.wrenchToThrust = np.zeros((len(self.thrusterData), 6))
        
        self.updateControlMatrix()
        rospy.loginfo("Initialized with "+ str(len(self.thrusterData)) + " thrusters")
    
    def updateControlMatrix(self):
        B = []
        
        for channel in sorted(self.thrusterData):
            rospy.loginfo("Adding thruster " + str(channel))
            if True or self.thrusterStatuses.has_key(channel) and self.thrusterStatuses[channel].thrusterOk:
                rospy.loginfo("Thruster " + str(channel) + " is ok")
                force = self.thrusterData[channel]["direction"]
                torque = np.cross(self.thrusterData[channel]["position"], self.thrusterData[channel]["direction"])
                wrench = np.hstack([force, torque]) #Row of values
                
                B.append(wrench) #List of rows (which need to be columns in the final matrix)
                
            else:
                B.append(np.zeros(6))
            
        self.thrustToWrench = np.transpose(np.array(B)) #Convert our list of rows to the transpose of the thrustToWrench then take the transpose of that
        self.wrenchToThrust = np.linalg.pinv(self.thrustToWrench) #This is where the magic happens. The pseudoinverse is like an inverse but works for non-square matricies
        
        print(self.thrustToWrench)
        print(self.wrenchToThrust)
    def commandCb(self, msg): #accept a twist msg (should be a wrench but same difference), then apply it to the thrusters
        desiredWrench = rosToWrench(msg)
        
        pinvOutput = np.matmul(self.wrenchToThrust, desiredWrench)
        
        
        rospy.loginfo(pinvOutput)
    def thrusterStatusCb(self, msg):
        needToUpdate = False #Flag set true if something happened that requires a control matrix update
        if not self.thrusterStatuses.has_key(msg.thrusterChannel): #If we've never had a message from this thruster before, update the control matrix
            needToUpdate = True
        elif not self.thrusterStatuses[msg.thrusterChannel].thrusterOk == msg.thrusterOk: #If this thruster's status has changed update the control matrix
            needToUpdate = True
            
        self.thrusterStatuses[msg.thrusterChannel] = msg #Store the updated thruster status message
        
        if needToUpdate:
            self.updateControlMatrix()
        
        
    def run(self): #Start spinning ros stuff.
        rospy.Subscriber("desiredThrustWrench", Wrench, self.commandCb)
        rospy.Subscriber("thrusterStatus", ThrusterStatus, self.thrusterStatusCb)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            for k, v in self.thrusterStatuses.items():
                if v.header.stamp < rospy.Time.now() + rospy.Duration(3): #3 second timeout for thrusters
                    pass
                    #self.thrusterStatuses[k].thrusterOk = False
            rate.sleep()
            
if __name__ == "__main__":
    rospy.init_node("vector_control")
    print("Starting up")
    node = VectorController()
    node.run()
