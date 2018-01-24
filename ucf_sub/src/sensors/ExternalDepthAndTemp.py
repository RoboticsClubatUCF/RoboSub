# created by Matthew Kurtz 1/23/2018

import ms5837 #A python module to interface with MS5837-30BA and MS5837-02BA waterproof pressure and temperature sensors. (https://github.com/bluerobotics/ms5837-python)
import rospy
import ms5837
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import FluidPressure

def external_depth_and_temp_sensor():
	
	sensor = ms5837.MS5837_30BA()
	
	#initialize sensor
	if not sensor.init()
		rospy.loginfo("external depth and temp sensor could not be initialized")
		exit(1)
	
	rospy.loginfo("starting external depth and temparture sensor")
	
	#node/publisher setup
	Tpub = rospy.Publisher('sensorData', Temperature) # check rostopic name (may be changed in future)
	Ppub = rospy.Publisher('sensorData', FluidPressure) # check rostopic name (may be changed in future)
	rospy.int_node('Bar30')
	rate = rospy.Rate(10) # parameter is in Hz, may need to be adjusted
	int counter = 0
	
	# enter loop
	while not rospy.is_shutdown():
		
		# conversions to make data work with ROS
		pressure = sensor.pressure() * 100000 # converts sensore data, measured in mBars to pascals
		temperature = sensor.temperature() # measured in Celsius
		
		# set up ROS temperature message
		Tmsg = Temperature()
		
		Tmsg.header.seq = counter
		Tmsg.header.stamp = rospy.Time.now()
		Tmsg.header.frame_id = "Temparature_id" # needs to be updated to correct id
		
		Tmsg.temperature = temperature
		Tmsg.variance = 0
		
		# set up ROS Pressure message
		Pmsg = FluidPressure()
		
		Pmsg.header.seq = counter
		Pmsg.header.stamp = rospy.Time.now()
		Pmsg.header.frame_id = "Pressure_id" # needs to be updated to correct id
		
		Pmsg.fluid_pressure = pressure
		Pmsg.variance = 0
		
		# publish pressure and temperature messages
		Tpub.publish(Tmsg)
		Ppub.publish(Pmsg)
		
		counter = counter + 1
		
		
def main():
	external_depth_and_temp_sensor():

if __name__ == "__main__":
	main()
		
		
		
		
		
	
