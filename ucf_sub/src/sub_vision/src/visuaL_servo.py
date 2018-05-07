import rospy
import image_geometry
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Wrench
import particle


class visual_servo:

	def __init__(self):
		self.particleNum = 1000
		self.imageWidth = 640
		self.imageHeight = 480
		self.xThreshold = 0
		self.yThreshold = 0
		self.lam = 1
		self.message = Wrench()
		self.particles == particle.initParticles(self.particleNum, self.imageHeight, self.imageWidth)
		self.thrusterPublisher = rospy.Publisher('desiredThrustWrench', Wrench, queue_size=1)
		self.box_sub = rospy.Subscriber("/object_bounding_box", numpy_msg(Floats), self.servoing)

	def interaction_matrix(coordinates):
		u = self.lam * (coordinates[0]/coordinates[2])
		v = self.lamb * (coordinates[1]/coordinates[2]) 
		Z = coordinates[2]
		return np.array([[-self.lam/Z, 0, u/Z, (u*v)/self.lam,-(self.lam + (math.square(u)/self.lam)), v], [0,-self.lam/Z,v/Z, self.lam + (math.square(v)/self.lam, -((u*v)/self.lam), -u]])

	def servoing(self, msg):

		#Find the camera position in the frame
		cX = image_geometry.cX
		cY = image_geometry.cY

		#If center of bounding box is not aligned with camera
		if math.fabs(msg[0]-cX) > self.xThreshold:
			if math.fabs(msg[1]-cY) > self.yThreshold:
				self.particles = particle.update(self.particles, (cX,cY))
				self.particles = particle.resample_particles(self.particles, self.particleNum)
				
				#Find new desired camera position
				newPosition = findBestParticle(self.particles)
				coordinates = (self.particles[newPosition][0],self.particles[newPosition][1])
				rectCoord = image_geometry.rectifyPoint(coordinates)
				newPosition = image_geometry.projectPixelTo3d(rectCoord)
				newCommand = np.matmul(np.linalg.pinv(interaction_matrix(newPosition,self.lamb)),rectCoord)

				#Create Wrench 
				#Todo: Figure out how to convert desired camera position to desired force and torque amounts
				message.force.x = newCommand[1]
				message.force.y = newCommand[0]
				message.force.z = newCommand[2]
				message.torque.x = newCommand[0]
				message.torque.y = newCommand[1]
				message.torque.z = newCommand[2]
				thrusterPublisher.publish(message)

def main(args):
	vs = visual_servo()
	rospy.init_node('visual_servo', anonymous=False)
	rospy.spin()
