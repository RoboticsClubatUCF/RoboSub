#!/usr/bin/env python3
import rospy
import struct
from pymodbus.client.sync import ModbusSerialClient as modbus
from pymodbus.exceptions import ModbusIOException
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, DiagnosticArray, DiagnosticStatus

def publish():
	sensor = modbus(method='rtu', port='/dev/ucfsub/depth', baudrate=115200)
	sensor.connect()

	rospy.init_node('Depth')
	tempPub = rospy.Publisher('ExternalTemperature', Temperature, queue_size=1)
	depthPub = rospy.Publisher('/depth', Float32, queue_size=1)
	posePub = rospy.Publisher('/depth/pose', PoseWithCovarianceStamped, queue_size=1)
	diagPub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

	temp = Temperature()
	depth = Float32()
	diag = DiagnosticArray()

	updateRate = rospy.get_param("/updateRate", 30)
	freq = rospy.Rate(updateRate)
	loop = 0

	pose = PoseWithCovarianceStamped()
	pose.header.frame_id = "odom"
	pose.pose.covariance = [0.0]*36
	pose.pose.covariance[14] = 0.01s
	pose.pose.pose.orientation.x = 0.0
	pose.pose.pose.orientation.y = 0.0
	pose.pose.pose.orientation.z = 0.0
	pose.pose.pose.orientation.w = 1.0
	pose.pose.pose.position.x = 0.0
	pose.pose.pose.position.y = 0.0

	while not rospy.is_shutdown():
		diag.status = [DiagnosticStatus(name='Leak', message=str(type(rr) is type(ModbusIOException), level=int(x == 'true')*2)]
		if loop >= updateRate:
			rr = sensor.read_holding_registers(address=8, count=2, unit=1)
			temp.temperature = struct.unpack('>f',struct.pack('>HH', *rr.registers))[0]
			tempPub.publish(temp)
			loop = 0
		loop += 1

		rr = sensor.read_holding_registers(address=2, count=2, unit=1)
		depth.data = 10.2*struct.unpack('>f',struct.pack('>HH', *rr.registers))[0]

		pose.pose.pose.position.z = depth.data
		pose.header.stamp = rospy.Time.now()
		posePub.publish(pose)
		depthPub.publish(depth)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
