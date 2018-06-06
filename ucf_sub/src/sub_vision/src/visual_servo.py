#import rospyself.server.start()
import image_geometry
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import particle
from sub_vision.msg import VisualServoAction, VisualServoGoal, VisualServoFeedback, VisualServoResult
import pulp

class visual_servo:

	def __init__(self,goal):
		self.server = actionlib.SimpleActionServer('visual_servo', VisualServoAction, self.execute, False)
		self.server.start()
		self.particleNum = 1000
		self.imageWidth = 640
		self.imageHeight = 480
		self.xThreshold = 0
		self.yThreshold = 0
		self.lam = 1
		self.message = Wrench()
		self.goal = goal
		self.response = TrackObjectResult()
		self.box_sub = rospy.Subscriber("/object_bounding_box", numpy_msg(Floats), self.servoing)
		if goal == VisualServoGoal.gate:
			self.particles == particle.initParticles(self.particleNum, self.imageHeight, self.imageWidth)
			self.thrusterPublisher = rospy.Publisher('desiredThrustWrench', Wrench, queue_size=1)
		elif goal == VisualServoGoal.pole:
			self.fx = image_geometry.fx
			self.fy = image_geometry.fy
			self.knownWidth = 0.0
			self.maintainedDistance = 12
			self.thrusterPublisher = rospy.Publisher('desiredThrustWrench', Wrench, queue_size=1)

	def interaction_matrix(self, dof, u, v, Z):
		int_matrix = np.array([[-self.lam/Z, 0, u/Z, (u*v)/self.lam,-(self.lam + (math.square(u)/self.lam)), v], [0,-self.lam/Z,v/Z, self.lam + math.square(v)/self.lam, -((u*v)/self.lam), -u]])
		return int_matrix[:,dof]

	def distance_to_object(self,perWidth):
		return (self.knownWidth * self.fx) / perWidth

	def servoing(self, msg):
		if self.goal == VisualServoGoal.pole:
			#Find distance to pole
			interaction = None
			coordinates = projectPixelTo3dRay(msg[0])
			u = self.lam * (coordinates[0]/coordinates[2])
			v = self.lamb * (coordinates[1]/coordinates[2])
			Z = coordinates[2]

			if distance_to_object(self.coordinates[1][0]-self.coordinates[0][0]) >= self.maintainedDistance:
				int_matrix = interaction_matrix([1,5], u,v,Z)
				forces = np.matmul(np.linalg.pinv(int_matrix), coordinates+self.translationUnits)
				message.force.x = None
				message.force.y = forces[0]
				message.force.z = None
				message.torque.x = None
				message.torque.y = None
				message.torque.z = forces[1]
				thrusterPublisher.publish(message)

			else:
				return None

		else:
		#Find the camera position in the frame
			cX = image_geometry.cX
			cY = image_geometry.cY

			#If center of bounding box is not aligned with camera
			if math.fabs(msg[0]-cX) > self.xThreshold:
				if math.fabs(msg[1]-cY) > self.yThreshold:
					coordinates = [image_geometry.projectPixelTo3dRay(msg[i]) for i in range(3)]
					u = [self.lam * (coordinates[i][0]/coordinates[i][2]) for i in range(3)]
					v = [self.lamb * (coordinates[i][1]/coordinates[i][2]) for i in range(3)]
					Z = [coordinates[i][2] for i in range(3)]
					dof = [0,1,2,3,4,5,6]
					int_matrix = interaction_matrix(dof, u[i], v[i], Z[i])

					for i in range(2):
						int_matrix = np.stack(int_matrix, interaction_matrix(dof, u[i+1], v[i+1], Z[i+1]))	


					self.particles = particle.update(self.particles, (cX,cY))
					self.particles = particle.resample_particles(self.particles, self.particleNum)
					#Find new desired camera position
					newPosition = findBestParticle(self.particles)
					coordinates = (self.particles[newPosition][0],self.particles[newPosition][1])
					newPosition = image_geometry.projectPixelTo3d(coordinates)
					newCommand = np.matmul(np.linalg.pinv(int_matrix),newPosition)

					#Create Wrench
					message.force.x = newCommand[1]
					message.force.y = newCommand[0]
					message.force.z = newCommand[2]
					message.torque.x = newCommand[3]
					message.torque.y = newCommand[4]
					message.torque.z = newCommand[5]
					thrusterPublisher.publish(message)
			else:
				self.response.success = True
				self.server.set_succeeded(self.response)

def start_servo(self, goal):
	vs = visual_servo(goal)
	rospy.init_node('visual_servo', anonymous=False)
	rospy.spin()

if __name__== "__main__":
	self.server = actionlib.SimpleActionServer('visual_servo', VisualServoAction, start_servo, False)
        self.server.start()
