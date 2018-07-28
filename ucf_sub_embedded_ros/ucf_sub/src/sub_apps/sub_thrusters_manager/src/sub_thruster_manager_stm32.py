#!/usr/bin/env python3
import rospy

import serial
import time
import struct
import binascii

from sub_trajectory.msg import ThrusterCmd, ThrusterStatus

class ThrusterManager:
    def __init__(self):
        port = rospy.get_param("/thrusters/port", default="/dev/ucfsub/thrusters")
        baud = rospy.get_param("/thrusters/baud", default=1000000)

        self.ser = serial.Serial(port, baud)
        self.ser.open()

    def encodeCommand(self, throttleValues):
        if len(throttleValues) != 8:
            return
        
        commandData = bytearray(struct.pack('hhhhhhhh', *[32767*x for x in throttleValues]))

        chk = 0
        for b in commandData:
            chk += b

        chk = chk % 256
        commandData.append(chk)
        
        zeroIndexes = [index for index,value in enumerate(commandData) if value == 0]
        for index, zeroIdx in enumerate(zeroIndexes):
            try:
                commandData[zeroIdx] = zeroIndexes[index+1] - zeroIdx
            except:
                commandData[zeroIdx] = len(commandData) - zeroIdx
        try:
            commandData = bytearray(struct.pack('B', zeroIndexes[0]+1)) + commandData + bytearray('\x00')
        except:
            commandData = bytearray(struct.pack('B', len(commandData)+1)) + commandData + bytearray('\x00')

        #print(commandData)
        return commandData

    def commandCb(self, msg):
        self.ser.write(self.encodeCommand(msg.cmd))