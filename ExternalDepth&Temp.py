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
	pub = rospy.Publisher('sensorData', ) # check rostopic name (may be changed in future
	rospy.int_node('Bar30')
	rate = rospy.Rate(10) # parameter is in Hz, may need to be adjusted
	
	# enter loop
	while not rospy.is_shutdown():
		
		# conversions to make data work with ROS
		Pressure = sensor.pressure() * 100000 # converts sensore data, measured in mBars to pascals
		Temperature = sensor.temperature() # measured in Celsius
		
		# set up ROS temperature message
		Tmsg = Temperature()
		
		Tmsg.header.seq = int(data[0])
		Tmsg.header.stamp = rospy.Time.now()
		Tmsg.header.frame_id = "Temparature_id" # needs to be updated to correct id
		
		Tmsg.temperature = Temperature
		Tmsg.variance = 0
		
		# set up ROS Pressure message
		Pmsg = FluidPressure()
		
		Pmsg.header.seq = int(data[
		Pmsg.header.stamp = rospy.Time.now()
		Pmsg.header.frame_id = "Pressure_id" # needs to be updated to correct id
		
		Pmsg.fluid_pressure = Pressure
		Pmsg.variance = 0
		
		# publish pressure and temperature messages
		pub.publish(Tmsg)
		pub.publish(Pmsg)
		
	
		
		
		
		
		
		
		
		
	
