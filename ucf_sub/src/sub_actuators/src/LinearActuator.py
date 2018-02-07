import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32
GPIO.setmode(GPIO.BOARD)
GPIO.setup(20, GPIO.OUT)
pwm = GPIO.PWM(20, 100)

def callback(data):
	pwm.ChangeDutyCycle(data.data*100)

def listen():
	rospy.Subscriber('LinearActuator', Float32, callback)
	rospy.init_node('LinearActuator', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	pwm.start(0)
	listen()
