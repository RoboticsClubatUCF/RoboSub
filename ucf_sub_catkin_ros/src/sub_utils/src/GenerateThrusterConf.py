#! /usr/bin/env python
import rospy
import tf
import time

from geometry_msgs.msg import Vector3Stamped

class Node:

    def __init__(self):
        self.tl = tf.TransformListener()
    
    def run(self):
        frames = [x for x in self.tl.getFrameStrings() if "thruster" in x]
        print(str(frames))
        
        for frame in frames:
            vec = Vector3Stamped()
            vec.header.frame_id = frame
            vec.vector.x = -1.0
            vec.vector.y = 0.0
            vec.vector.z = 0.0
            
            newVec = self.tl.transformVector3("base_link", vec)
            print("Item " + frame + " has direction \n" + str(newVec.vector))

if __name__ == '__main__':
    rospy.init_node('thruster_configure')
    node = Node()
    start = time.time()
    while time.time() < start + 5:
        pass
    
    node.run()
