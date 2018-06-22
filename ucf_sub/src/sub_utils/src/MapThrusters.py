#!/usr/bin/env python3
import rospy
import yaml
import threading
import sys

from sub_trajectory.msg import ThrusterCmd

rospy.init_node('thruster_configure')
pub = rospy.Publisher('/thrusters/cmd_vel', ThrusterCmd, queue_size=10)
msg = ThrusterCmd()

def publish():
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()
        
t = threading.Thread(target=publish)
t.start()

print("Generating mapping for Robosub thrusters")
print("There are 8 thruster positions")
print("FLH FRH BRH BLH\nFLV FRV BRV BLV")
print("First letter is Front/Back")
print("Second letter is Left/Right")
print("Third letter is Vertical/Horizontal")
print("#-------------------------------------#")
print("The thruster direction is either W or N")
print("W means fluid is exiting from the Wire side")
print("N means fluid is exiting from the Nozzle side")
print("#-------------------------------------#")
print("This utility will iterate through all the thrusters")
print("And ask which is firing and in what direction")

thrusterDict = {}

for i in range(8):
    msg.cmd = [0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0]
    msg.cmd[i] = 0.1
    
    retry = 0
    positionString = "A"
    while (positionString not in "FLH FRH BRH BLH FLV FRV BRV BLV") or len(positionString) != 3:
        positionString = input("Enter the position designation of the active thruster:").upper()
        if positionString not in "FLH FRH BRH BLH FLV FRV BRV BLV" or len(positionString) != 3:
            print("Possible values: FLH FRH BRH BLH FLV FRV BRV BLV")
            retry += 1
        if retry > 10:
            sys.exit()
    
    retry = 0        
    directionString = "A"
    while (directionString not in "WN") or len(directionString) != 1:
         directionString = input("Enter the side fluid is exiting from:").upper()
         if directionString not in "WN" or len(directionString) != 1:
             print("Possible values: W N")
             retry += 1
         if retry > 10:
            sys.exit()    
    
    thrusterData = {}
    thrusterData["thruster_name"] = positionString
    thrusterData["thruster_frame"] = positionString + "_Thruster"
    thrusterData["thruster_type"] = "t200" #TODO: Support non t200 thrusters
    thrusterData["max_output"] = 6.6 #TODO: check units
    thrusterData["data_frame"] = "base_link"
    
    position = [0.0, 0.0, 0.0]
    direction = [0.0, 0.0, 0.0]
    
    if "F" in positionString:
        position[0] = 0.1
    elif "B" in positionString:
        position[0] = -0.1
    
    if "L" in positionString:
        position[1] = -0.1
    elif "R" in positionString:
        position[1] = 0.1
        
    
    if "V" in positionString:
        if directionString == "W":
            direction = [0.0,0.0,-1.0]
        elif directionString == "N":
            direction = [0.0,0.0,1.0]
    elif "H" in positionString:
        if "F" in positionString:
            if directionString == "W":
                direction[0] = 0.707
            elif directionString == "N":
                direction[0] = -0.707
            
        elif "B" in positionString:
            if directionString == "W":
                direction[0] = -0.707
            elif directionString == "N":
                direction[0] = 0.707
    
        if "L" in positionString:
            if directionString == "W":
                direction[1] = 0.707
            elif directionString == "N":
                direction[1] = -0.707
            
        elif "R" in positionString:
            if directionString == "W":
                direction[1] = -0.707
            elif directionString == "N":
                direction[1] = 0.707                  
    
    thrusterData["position"] = position
    thrusterData["direction"] = direction
    
    thrusterDict[i] = thrusterData

msg.cmd = [0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0]

with open("high_level_params.param", "w") as outfile:
    yaml.dump({"thrusters":thrusterDict},outfile)
    
