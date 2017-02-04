#! /usr/bin/env python
import rospy
import time

from sub_trajectory.msg import ThrusterStatus
from geometry_msgs.msg import Wrench
from std_msgs.msg import Header

import numpy as np

def rosToArray(msg): #Convert a ros message with 1-4 dimensions into a numpy array
    return np.array([getattr(msg, key) for key in ("x", "y", "z", "w") if hasattr(msg, key)])
    
def rosToWrench(msg): #convert a ros message with a force and torque vector into a numpy array
    return np.hstack((rosToArray(msg.force), rosToArray(msg.torque)))

class VectorThrustTester:
    def __init__(self):
        self.statusPub = rospy.Publisher("thrusterStatus", ThrusterStatus, queue_size=10)
        self.commandPub = rospy.Publisher("desiredThrustWrench", Wrench, queue_size=10)
        
    def run(self):
        rate = rospy.Rate(30)
        
        while self.statusPub.get_num_connections() == 0:
            rate.sleep()
        while not rospy.is_shutdown():
            thrusterStatus = ThrusterStatus()
            header = Header()
            header.stamp = rospy.get_rostime()
            thrusterStatus.header = header
            thrusterStatus.thrusterOk = True
            
            for channel in range(0,8):
                thrusterStatus.thrusterChannel = channel
                self.statusPub.publish(thrusterStatus)
                rate.sleep()
                #rospy.loginfo("Publish" + str(channel))
            
            #time.sleep(1)
            
            testWrench = Wrench()
            
            testWrench.force.x = 0.0
            testWrench.force.y = 0.0
            testWrench.force.z = -30.0
            testWrench.torque.x = 0.0
            testWrench.torque.y = 0.0
            testWrench.torque.z = 0.0
            
            #self.commandPub.publish(testWrench)
            
            rate.sleep()
        
        #break
        while not rospy.is_shutdown():
            rate.sleep()
if __name__ == "__main__":
    rospy.init_node("test_control")
    print("Starting up")
    node = VectorThrustTester()
    node.run()
