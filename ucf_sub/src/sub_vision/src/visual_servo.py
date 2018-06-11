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

		self.vision_client = actionlib.SimpleActionClient('track_object', TrackObjectAction)
		self.vision_client.wait_for_server()

		self.camera_info = None

		self.result=False
		self.camera_info_pub = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, self.initInfo)
		self.vision_feedback = rospy.Subscriber('/track_object/feedback', TrackObjectFeedback, self.servoing)
        self.thruster_pub = rospy.Publisher('/autonomyWrench', Wrench, queue_size=1)
        self.thruster_sub = rospy.Subscriber("/autonomyWrench", Wrench, self.republishWrench)
        self.desired_wrench = rospy.Publisher("/desiredThrustWrench", Wrench, queue_size=1)

		self.startYaw = None
        self.FirstIMUCall = True
        self.imuSubscriber = rospy.Subscriber("/imu/data", Imu, self.initIMU)
        self.depthSubscriber = rospy.Subscriber("/Depth", Float32, self.initDepth)
        self.knownWidth = 0.0
        self.maintainedDistance = 12
        self.speed = 1



	def initInfo(self, msg):
		self.camera_info = ig.PinholeCameraModel()
		self.camera_info.fromCameraInfo(msg)
        self.fx = self.camera_info.fx()
        self.fy = self.camera_info.fy()


	def interaction_matrix(self, dof, u, v, Z):
		int_matrix = np.array([[-self.lam/Z, 0, u/Z, (u*v)/self.lam,-(self.lam + (math.square(u)/self.lam)), v], [0,-self.lam/Z,v/Z, self.lam + math.square(v)/self.lam, -((u*v)/self.lam), -u]])
		return int_matrix[:,dof]

	def distance_to_object(self,perWidth):
		return (self.knownWidth * self.fx) / perWidth

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

 		if msg.servotask == VisualServoGoal.gate:
 		    self.goal = TrackObjectGoal()
            self.goal.objectType = self.goal.startGate
			self.goal.servoing = True
			self.vision_client.send_goal(self.goal)


        elif msg.servotask == VisualServoGoal.drift:
          	self.startYaw = self.orientationZ
          	self.goal = TrackObjectGoal()
			self.goal.servoing = True
            self.goal.objectType = self.goal.pole
        	self.vision_client.send_goal(self.goal)

		elif msg.servotask == VisualServoGoal.align:
	    	self.goal = TrackObjectGoal()
			self.goal.servoing = True
            self.goal.objectType = self.goal.pole
            self.vision_client.send_goal(self.goal)

		while self.running:
	        if self.server.is_preempt_requested() or self.server.is_new_goal_available():
        	    self.running = False
                continue


		self.response.aligned = False
		self.server.set_succeeded(self.response)

	def servoing(self,msg):
		if self.goal == VisualServoGoal.drift:
			if self.orientationZ - self.startYaw < 270:
				#Find distance to pole
				interaction = None
				coordinates = self.camera_info.projectPixelTo3dRay(msg.feedback.center)
				u = self.lam * (coordinates[0]/coordinates[2])
				v = self.lamb * (coordinates[1]/coordinates[2])
				Z = coordinates[2]

				if self.distance_to_object(msg.width) >= self.maintainedDistance:
					int_matrix = self.interaction_matrix([1,5], u,v,Z)
					forces = np.matmul(np.linalg.pinv(int_matrix), coordinates+self.translationUnits)
					self.message.force.x = self.speed
					self.message.force.y = forces[0]
					self.message.force.z = self.Depth
					self.message.torque.x = self.orientationX
					self.message.torque.y = self.orientationY
					self.message.torque.z = forces[1]
					self.thruster_pub.publish(message)

			else:
				self.response.aligned = True
				self.server.set_succeeded(self.response)

		elif self.goal == VisualServoGoal.gate or self.goal == VisualServoGoal.align:
			features = []
			features[0] = (msg.feedback.center[0]-(msg.feedback.width/2),msg.feedback.center[1]-(msg.feedback.height/2))
			features[1] = (features[0][0]+msg.feedback.width, features[0][1])
			features[2] = (features[0][0], features[0][1]+msg.feedback.height)


			#Find the camera position in the frame
			cX = self.camera_info.cx()
			cY = self.camera_info.cy()

			#If center of bounding box is not aligned with camera
			if math.fabs(msg.feedback.center[0]-cX) > self.xThreshold:
				if math.fabs(msg.feedback.center[1]-cY) > self.yThreshold:
					coordinates = [self.camera_info.projectPixelTo3dRay(features[i]) for i in range(3)]
					rospy.loginfo(coordinates)
					u = [self.lam * (coordinates[i][0]/coordinates[i][2]) for i in range(3)]
					v = [self.lam * (coordinates[i][1]/coordinates[i][2]) for i in range(3)]
					Z = [coordinates[i][2] for i in range(3)]
					dof = [0,1,2,3,4,5,6]
					int_matrix = self.interaction_matrix(dof, u[0], v[0], Z[0])

					for i in range(2):
						int_matrix = np.stack(int_matrix, self.interaction_matrix(dof, u[i+1], v[i+1], Z[i+1]))


					rospy.loginfo(int_matrix)
					newCommand = np.matmul(np.linalg.pinv(int_matrix), (cX, cY))

					#Create Wrench
					self.message.force.x = newCommand[1]
					self.message.force.y = newCommand[0]
					self.message.force.z = newCommand[2]
					self.message.torque.x = newCommand[3]
					self.message.torque.y = newCommand[4]
					self.message.torque.z = newCommand[5]
					self.thruster_pub.publish(message)

			else:
				self.response.aligned = True
				self.server.set_succeeded(self.response)


if __name__== "__main__":
        rospy.init_node('visual_servo', anonymous=False)
	vs = visual_servo()
        rospy.spin()

