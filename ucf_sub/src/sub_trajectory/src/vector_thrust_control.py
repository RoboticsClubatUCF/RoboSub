#! /usr/bin/env python
import rospy

from sub_trajectory.msg import ThrusterStatus, ThrusterCmd
from geometry_msgs.msg import Wrench

import numpy as np
from scipy.optimize import minimize

def rosToArray(msg): #Convert a ros message with 1-4 dimensions into a numpy array
    return np.array([getattr(msg, key) for key in ("x", "y", "z", "w") if hasattr(msg, key)])
    
def rosToWrench(msg): #convert a ros message with a force and torque vector into a numpy array
    return np.hstack((rosToArray(msg.force), rosToArray(msg.torque)))
    
class VectorController:

    def __init__(self): #Get parameters from rosparam/URDF/TF, get thruster objects ready
        self.thrusterStatuses = dict()
        self.thrusterData = {int(k):v for k,v in rospy.get_param("/thrusters").items()}
        
        self.thrustToWrench = np.zeros((6, max(sorted(self.thrusterData))+1))
        self.wrenchToThrust = np.zeros((max(sorted(self.thrusterData))+1, 6))
        
        self.updateControlMatrix()
        rospy.loginfo("Initialized with "+ str(len(self.thrusterData)) + " thrusters")
        
   
    def updateControlMatrix(self):
        B = []
        
        for channel in sorted(self.thrusterData):
            #rospy.loginfo("Adding thruster " + str(channel))
            if self.thrusterStatuses.has_key(channel) and self.thrusterStatuses[channel].thrusterOk:
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
        
        #Log the latest control matricies at most every 3 seconds
        rospy.logdebug("\n" + str(self.thrustToWrench))
        rospy.logdebug("\n" + str(self.wrenchToThrust))
        
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
        desiredWrench = rosToWrench(msg)
        
        #rospy.loginfo("DesiredWrench: " + str(desiredWrench))
        pinvOutput = self.wrenchToThrust.dot(desiredWrench)
        #rospy.loginfo("pinv: " + str(pinvOutput))
        #optimization stuff is based on CUAUV and UF's methods
        optimize = False
        for v in pinvOutput:
            if -1 > v or 1 < v:
                optimize = True
                break
                
        if np.linalg.norm(desiredWrench - self.thrustToWrench.dot(pinvOutput)) > 0.01:
            optimize = True
            
        #rospy.loginfo("Error: " + str(np.linalg.norm(desiredWrench - self.thrustToWrench.dot(pinvOutput))))
        
        #So we use this to apply a different set of power coefficients in order to better fit the data from bluerobotics
        #I have no idea if this causes stability issues or something but /shrug
        def powerCost(u):
            powerCost = 0
            for i in sorted(self.thrusterData):
                j = 0 if u[i] > 0 else 1
                powerCost +=  self.A[i][j] * u[i] * u[i] + self.B[i][j] * u[i]#apply a polynomial from bluerobotics data for the t200 power consumption
                
            return powerCost * 0.01
            
        def powerJacobian(u):
            powerJac = np.array(u)
            
            for i in sorted(self.thrusterData):
                j = 0 if u[i] > 0 else 1
                powerJac[i] = self.A[i][j] * u[i] + self.B[i][j]
                
            return powerJac * 0.01
            
        def objective(u):
            #Use UF's method here for our cost function but we'll probably add some factor to weight error vs effort
            errorCost = np.linalg.norm(self.thrustToWrench.dot(u) - desiredWrench) ** 2 #sum of the squares of the errors in the output wrench
            
            #Returns a single cost number
            return errorCost + powerCost(u) #TODO relative importance factor?
            
        def jacobian(u):
            errorJacobian = 2 * self.thrustToWrench.T.dot(self.thrustToWrench.dot(u)-desiredWrench) #2 times thruster matrix times wrench error
            #powerJacobian = 2 * self.A.dot(u) + self.B #Derivative of power polynomial
            
            #Returns an array of the partial derivatives at point u
            return errorJacobian + powerJacobian(u)
            
        if optimize:
            #TODO: SLSQP vs other methods?
            #TODO: per thruster bounds? (Good for when we kill the t200s and have to slap on seabotix)
            #TODO: Impact of tol and x0 on runtime?
            #TODO: what happens if x0 is out of bounds?
             minimized = minimize(
                fun=objective,
                x0=pinvOutput/(np.linalg.norm(pinvOutput)+0.1),
                method="SLSQP",
                jac=jacobian,
                bounds=[(-1,1) for x in self.thrusterData],
                tol=0.1)
             rospy.logdebug("Optimized error: " + str(np.linalg.norm(desiredWrench - self.thrustToWrench.dot(minimized.x))))
        #rospy.loginfo("pinv: " + str(pinvOutput))
        
        message = ThrusterCmd()
        
        if optimize:
            rospy.loginfo("optimize output wrench: " + str(self.thrustToWrench.dot(minimized.x)))
            message.cmd = minimized.x.tolist()
        else:
            rospy.loginfo("pinv output wrench: " + str(self.thrustToWrench.dot(pinvOutput)))
            message.cmd = pinvOutput.tolist()
        
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
        rospy.Subscriber("desiredThrustWrench", Wrench, self.commandCb)
        rospy.Subscriber("thrusterStatus", ThrusterStatus, self.thrusterStatusCb)
        
        self.thrustPublisher = rospy.Publisher("thrusterCmd", ThrusterCmd, queue_size=10)
        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            for k, v in self.thrusterStatuses.items():
                if v.header.stamp + rospy.Duration(3, 0) < rospy.get_rostime() and self.thrusterStatuses[k].thrusterOk: #3 second timeout for thrusters
                    rospy.logwarn("Thruster " + str(k) + " timed out for " + str(rospy.get_rostime() - v.header.stamp))
                    self.thrusterStatuses[k].thrusterOk = False
            rate.sleep()
            
if __name__ == "__main__":
    rospy.init_node("vector_control")
    print("Starting up")
    node = VectorController()
    node.run()
