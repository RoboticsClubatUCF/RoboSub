#! /usr/bin/env python
import rospy
import image_geometry as ig
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import particle
import vision_manager
from sub_vision.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult, TrackObjectAction, TrackObjectGoal, TrackObjectFeedback, TrackObjectResult
from geometry_msgs.msg import Wrench
import actionlib
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Float32
import math
import numpy as np

class visual_servo:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('visual_servo', VisualServoAction,self.execute,False)
        self.server.start()

        self.xThreshold = 0
        self.yThreshold = 0

        self.lam = 1

        self.message = Wrench()

        self.response = VisualServoResult()

        self.goal = VisualServoGoal()

        self.vision_client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
        self.vision_client.wait_for_server()

        self.camera_info = None

        self.result=False

        self.camera_info = ig.PinholeCameraModel()

        self.knownWidth = 3.5
        self.maintainedDistance = 12
        self.speed = 1

        self.startYaw = None
        self.FirstIMUCall = True

        self.camera_info_pub = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, self.initInfo)
        self.vision_feedback = rospy.Subscriber('/track_object/feedback', TrackObjectFeedback, self.servoing)
        self.thruster_pub = rospy.Publisher('/autonomyWrench', Wrench, queue_size=1)
        self.thruster_sub = rospy.Subscriber("/autonomyWrench", Wrench, self.republishWrench)
        self.desired_wrench = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=1)

        self.imuSubscriber = rospy.Subscriber("/imu/data", Imu, self.initIMU)
        self.depthSubscriber = rospy.Subscriber("/Depth", Float32, self.initDepth)

    def initInfo(self, msg):
        self.camera_info.fromCameraInfo(msg)
        self.fx = self.camera_info.fx()
        self.fy = self.camera_info.fy()


    def interaction_matrix(self, dof, u, v):
        int_matrix = np.array([[-self.lam, 0, u, (u*v)/self.lam,-(self.lam + (np.square(u)/self.lam)), v], [0,-self.lam,v, self.lam + np.square(v)/self.lam, -((u*v)/self.lam), -u]])
        #return int_matrix[:,dof]
        return int_matrix

    def distance_to_object(self,perWidth):
        distance = (self.knownWidth * self.fx) / perWidth
        print(distance)
        return distance

    def initIMU(self, msg):
        self.orientationX = msg.orientation.x
        self.orientationY = msg.orientation.y
        self.orientationZ = msg.orientation.z

    def republishWrench(self, msg):
        self.desired_wrench.publish(msg)

    def initDepth(self, msg):
        self.Depth = msg

    def execute(self,msg):
        self.running=True
        self.goal = msg
        if msg.servotask == VisualServoGoal.gate:
            goal = TrackObjectGoal()
            goal.objectType = goal.startGate
            goal.servoing = True
            self.vision_client.send_goal(goal)

        elif msg.servotask == VisualServoGoal.drift:
            self.startYaw = self.orientationZ
            goal = TrackObjectGoal()
            goal.servoing = True
            goal.objectType = goal.pole
            self.vision_client.send_goal(goal)

        elif msg.servotask == VisualServoGoal.align:
            goal = TrackObjectGoal()
            goal.servoing = True
            goal.objectType = goal.pole
            self.vision_client.send_goal(goal)

        while self.running:
            if self.server.is_preempt_requested() or self.server.is_new_goal_available():
                self.running = False
                continue


        self.response.aligned = False
        self.server.set_succeeded(self.response)

    def servoing(self,msg):
        if self.goal.servotask == VisualServoGoal.drift:
            if self.orientationZ - self.startYaw < 270:
                x = msg.feedback.center[0]
                y = msg.feedback.center[1]
                coordinates = np.array([x,y])
                #Find distance to pole
                interaction = None
                #coordinates = self.camera_info.projectPixelTo3dRay(msg.feedback.center)
                #u = self.lam * (coordinates[0]/coordinates[2])
                #v = self.lamb * (coordinates[1]/coordinates[2])
                #Z = coordinates[2]
                #print(msg.width)
                if self.distance_to_object(msg.width) >= self.maintainedDistance:
                    int_matrix = self.interaction_matrix([1,5], x,y)
                    int_matrix = int_matrix[:,[1,5]]
                    forces = np.matmul(np.linalg.pinv(int_matrix), coordinates+self.translationUnits)
                    self.message.force.x = self.speed
                    self.message.force.y = forces[0]
                    self.message.force.z = self.Depth
                    self.message.torque.x = self.orientationX
                    self.message.torque.y = self.orientationY
                    self.message.torque.z = forces[1]
                    self.thruster_pub.publish(self.message)

            else:
                self.response.aligned = True
                self.server.set_succeeded(self.response)

        else:
            features = []
            width = msg.feedback.width/2
            print(self.distance_to_object(width))
            height = msg.feedback.height/2
            features.append((msg.feedback.center[0]-width,msg.feedback.center[1]-height))
            features.append((msg.feedback.center[0]+width, features[0][1]))
            features.append((features[0][0], msg.feedback.center[1]+height))


            #Find the camera position in the frame
            if self.camera_info is not None:
                cX = self.camera_info.cx()
            if self.camera_info is not None:
                cY = self.camera_info.cy()

            #If center of bounding box is not aligned with camera
            if math.fabs(msg.feedback.center[0]-cX) > self.xThreshold:
                if math.fabs(msg.feedback.center[1]-cY) > self.yThreshold:
                    #rectified = self.camera_info.rectifyPoint(features[0])
                    #coordinate = ig.projectPixelTo3dRay(features[0])
                    #coordinates = [self.camera_info.projectPixelTo3dRay(features[i]) for i in range(3)]
                    #rospy.loginfo(coordinate)
                    #u = [self.lam * (coordinates[i][0]/coordinates[i][2]) for i in range(3)]
                    #v = [self.lam * (coordinates[i][1]/coordinates[i][2]) for i in range(3)]
                    #Z = [coordinates[i][2] for i in range(3)]
                    dof = [0,1,2,3,4,5,6]
                    int_matrix = self.interaction_matrix(dof, features[0][0], features[0][1])

                    for i in range(2):
                        #rospy.loginfo(int_matrix)
                        int_matrix = np.vstack((int_matrix, self.interaction_matrix(dof, features[i+1][0], features[i+1][1])))

                    newPos = np.array([[cX-width],[cY-height], [cX+width], [cY-height], [cX-width], [cY+height]])
                    newCommand = np.matmul(np.linalg.pinv(int_matrix), newPos)

                    #Create Wrench
                    self.message.force.x = newCommand[1]
                    self.message.force.y = newCommand[0]
                    self.message.force.z = newCommand[2]
                    self.message.torque.x = newCommand[3]
                    self.message.torque.y = newCommand[4]
                    self.message.torque.z = newCommand[5]
                    self.thruster_pub.publish(self.message)

            else:
                self.response.aligned = True
                self.server.set_succeeded(self.response)


if __name__== "__main__":
    rospy.init_node('visual_servo', anonymous=False)
    vs = visual_servo()
    rospy.spin()

