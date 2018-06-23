import gpio
import rospy
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

CHANNEL1 = 394
CHANNEL2 = 395
GPIO.setup(CHANNEL1, gpio.OUT)
GPIO.setup(CHANNEL2, gpio.OUT)
freq = rospy.Rate(10)

def fireTorp1(data):
	gpio.output(CHANNEL1, True)
	freq.sleep()
	gpio.output(CHANNEL1, False)

def fireTorp2(data):
	gpio.output(CHANNEL2, True)
	freq.sleep()
	gpio.output(CHANNEL2, False)

def start():
	rospy.Subscriber('Torpedo1', Bool, fireTorp1)
	rospy.Subscriber('Torpedo2', Bool, fireTorp2)
	rospy.init_node('TorpedoWatcher', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass
