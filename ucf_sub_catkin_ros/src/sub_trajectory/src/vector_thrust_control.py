#! /usr/bin/env python
import rospy

from sub_trajectory.msg import ThrusterStatus, ThrusterCmd
from geometry_msgs.msg import WrenchStamped

import numpy as np
import math, time
from scipy.optimize import minimize

from dynamic_reconfigure.server import Server
from sub_trajectory.cfg import VectorConfig

def rosToArray(msg): #Convert a ros message with 1-4 dimensions into a numpy array
    return np.array([getattr(msg, key) for key in ("x", "y", "z", "w") if hasattr(msg, key)])
    
def rosToWrench(msg): #convert a ros message with a force and torque vector into a numpy array
    return np.hstack((rosToArray(msg.force), rosToArray(msg.torque)))
    
class VectorController:

    def __init__(self): #Get parameters from rosparam/URDF/TF, get thruster objects ready
        self.thrusterStatuses = {int(k):True for k in rospy.get_param("/thrusters")}
        self.thrusterData = {int(k):v for k,v in rospy.get_param("/thrusters").items()}
        
        self.thrustToWrench = np.zeros((6, max(sorted(self.thrusterData))+1))
        self.wrenchToThrust = np.zeros((max(sorted(self.thrusterData))+1, 6))
        
        self.updateControlMatrix()
        rospy.loginfo("Initialized with "+ str(len(self.thrusterData)) + " thrusters")

        self.reconfigureServer = Server(VectorConfig, self.reconfigureCallback)
        self.thrust_limit = 0.2
        
    def reconfigureCallback(self, config, level):
        self.thrust_limit = config["thrust_limit"]
        for k in config:
            if k.startswith("thruster"):
                self.thrusterStatuses[int(k[13])] = config[k]
        
        self.updateControlMatrix()
        return config

    def updateControlMatrix(self):
        B = []
        
        for channel in sorted(self.thrusterData):
            #rospy.loginfo("Adding thruster " + str(channel))
            if self.thrusterStatuses[channel]:
                #rospy.loginfo("Thruster " + str(channel) + " is ok")
                
                #TODO: should we multiply by the actual thrust limit? I think so
                force = np.array(self.thrusterData[channel]["direction"]) * self.thrusterData[channel]["max_output"]
                rospy.logdebug(force)
                torque = np.cross(self.thrusterData[channel]["position"], force)
                wrench = np.hstack([force, torque]) #Row of values
                
                B.append(wrench) #List of rows (which need to be columns in the final matrix)
                
            else:
                B.append(np.zeros(6))
            
        self.thrustToWrench = np.transpose(np.array(B)) #Convert our list of rows to the transpose of the thrustToWrench then take the transpose of that
        self.wrenchToThrust = np.linalg.pinv(self.thrustToWrench) #This is where the magic happens. The pseudoinverse is like an inverse but works for non-square matricies
        
        self.setupPowerCoeffs()
    
    def setupPowerCoeffs(self): #Set up arrays of coefficients for thruster power measured as Ax^2 + Bx
        self.A = np.zeros((max(sorted(self.thrusterData))+1,2))
        self.B = np.zeros((max(sorted(self.thrusterData))+1,2))
        
        for channel in sorted(self.thrusterData):
            if self.thrusterData[channel]["thruster_type"] == "t200": #Add coefficients for t200 thrusters
                self.A[channel][0] = 0.15 #Coefficients for forward thust
                self.B[channel][0] = 0.77
                
                self.A[channel][1] = 0.197 #Coefficients for reverse thrust
                self.B[channel][1] = -1.0
                
                #TODO: add seabotix?
            else:
                raise NotImplementedError
                
    def commandCb(self, msg): #accept a wrench msg, then apply it to the thrusters
        desiredWrench = rosToWrench(msg.wrench)
        
        #rospy.loginfo("DesiredWrench: " + str(desiredWrench))
        pinvOutput = self.wrenchToThrust.dot(desiredWrench)
        #rospy.loginfo("pinv: " + str(pinvOutput))
        #optimization stuff is based on CUAUV and UF's methods
        optimize = False
        for v in pinvOutput:
            if -self.thrust_limit > v or self.thrust_limit < v:
                optimize = True
                break
                
        if np.linalg.norm(desiredWrench - self.thrustToWrench.dot(pinvOutput)) > 0.01:
            optimize = True
            
        
        #So we use this to apply a different set of power coefficients in order to better fit the data from bluerobotics
        #I have no idea if this causes stability issues or something but /shrug
        def powerCost(u):
            powerCost = 0
            for i in sorted(self.thrusterData):
                j = 0 if u[i] > 0 else 1
                powerCost +=  self.A[i][j] * u[i] * u[i] + self.B[i][j] * u[i]#apply a polynomial from bluerobotics data for the t200 power consumption
                
            return powerCost * 0.001
            
        #Optimizer objective function that tries to minimize the difference in angle between the desired and actual wrench, then the magnitude, then the power usage
        def objective(u):
            #Originally based off of UF's cost function but changed to consider the direction of the output as more important
            errorCost = 0.01 * np.linalg.norm(self.thrustToWrench.dot(u) - desiredWrench) #sum of the squares of the errors in the output wrench
            
            #Extend things a little by also considering the angle of the error
            #This seems to only improve things when tol is rather high
            if abs((np.linalg.norm(self.thrustToWrench.dot(u))*np.linalg.norm(desiredWrench))) > 0.001:
                errorCost -= abs(np.inner(self.thrustToWrench.dot(u),desiredWrench)/(np.linalg.norm(self.thrustToWrench.dot(u))*np.linalg.norm(desiredWrench)))
            #Returns a single cost number
            return errorCost + powerCost(u) #TODO relative importance factor?
            
        if optimize:
            #TODO: SLSQP vs other methods?
            #TODO: per thruster bounds? (Good for when we kill the t200s and have to slap on seabotix)
            #tol should be low enough to make it actually have to optimize
            #TODO: tol/x0 on accuracy and runtime
            #What happens if x0 is out of bounds? A: The optimizer ignores the bounds
            minimized = minimize(
                fun=objective,
                x0=self.thrust_limit*pinvOutput/np.linalg.norm(pinvOutput),
                method="SLSQP",
                jac=False,#jacobian,
                bounds=[(-self.thrust_limit,self.thrust_limit) for _ in self.thrusterData],
                tol=0.0001)
        
        message = ThrusterCmd()
        actualMsg = WrenchStamped()
        actual = None
        if optimize:
            #rospy.loginfo("optimize output wrench: " + str(self.thrustToWrench.dot(minimized.x)))
            message.cmd = minimized.x.tolist()
            actual = self.thrustToWrench.dot(minimized.x)
        else:
            #rospy.loginfo("pinv output wrench: " + str(self.thrustToWrench.dot(pinvOutput)))
            message.cmd = pinvOutput.tolist()
            actual = self.thrustToWrench.dot(pinvOutput)
        actualMsg.wrench.force.x = actual[0]
        actualMsg.wrench.force.y = actual[1]
        actualMsg.wrench.force.z = actual[2]
        actualMsg.wrench.torque.x = actual[3]
        actualMsg.wrench.torque.y = actual[4]
        actualMsg.wrench.torque.z = actual[5]
        actualMsg.header.stamp = rospy.Time.now()
        
        self.actualWrenchPub.publish(actualMsg)
        self.thrustPublisher.publish(message)

    def thrusterStatusCb(self, msg):
        needToUpdate = False #Flag set true if something happened that requires a control matrix update
        if not self.thrusterStatuses.has_key(msg.thrusterChannel): #If we've never had a message from this thruster before, update the control matrix
            needToUpdate = True
        elif not self.thrusterStatuses[msg.thrusterChannel].thrusterOk == msg.thrusterOk: #If this thruster's status has changed update the control matrix
            needToUpdate = True
            
        self.thrusterStatuses[msg.thrusterChannel] = msg #Store the updated thruster status message
        
        if needToUpdate:
            rospy.logdebug("update")
            self.updateControlMatrix()
        else:
            rospy.logdebug("Dont update")
        
        
    def run(self): #Start spinning ros stuff.
	self.thrustPublisher = rospy.Publisher("/thrusters/cmd_vel", ThrusterCmd, queue_size=10)
        self.actualWrenchPub = rospy.Publisher("actualThrustWrench", WrenchStamped, queue_size=10)

        rospy.Subscriber("desiredThrustWrench", WrenchStamped, self.commandCb)
        rospy.Subscriber("thrusterStatus", ThrusterStatus, self.thrusterStatusCb)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            for k, v in self.thrusterStatuses.items():
                #if v.header.stamp + rospy.Duration(3, 0) < rospy.get_rostime() and self.thrusterStatuses[k].thrusterOk: #3 second timeout for thrusters
                    pass
                    #rospy.logwarn("Thruster " + str(k) + " timed out for " + str(rospy.get_rostime() - v.header.stamp))
                    #self.thrusterStatuses[k].thrusterOk = False
            rate.sleep()
            
if __name__ == "__main__":
    rospy.init_node("vector_control")
    print("Starting up")
    node = VectorController()
    node.run()
