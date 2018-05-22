#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge, CvBridgeError
import time


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_overlay",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/stereo/left/image_raw",Image,self.callback)
    self.twistJoystickForce = None
    self.twistJoystickTorque = None
    trans_sub = rospy.subscribe("/translate/joy", Joy, self.translateCb);
    rot_sub = rospy.subscribe("/rotate/joy", Joy, self.rotateCb);


  def translateCb(self,msg):
    twistMsg.force.x = msg.axes[1] * 20
    twistMsg.force.y = msg.axes[0] * 20
    twistMsg.force.z = msg.axes[2] * 20
    twistJoystickForce = [twistMsg.force.x,twistMsg.y,twistMsg.z]  

  def rotateCb(self, msg):
    twistMsg.torque.x = msg.axes[0] * -2
    twistMsg.torque.y = msg.axes[1] * 2
    twistMsg.torque.z = msg.axes[2] * 2
    twistJoystickTorque = [twistMsg.torque.x,twistMsg.torque.y,twistMsg.torque.z]

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    font = cv2.FONT_HERSHEY_SIMPLEX
    text = str(time.time())
     
    pts = np.array([[50,430],[50,380],[100,430],[100,380]], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(img, [pts], True, (0,255,255))
    cv2.line(img, [75,405], [twistJoystickForce.x, TwistJoystickForce.y], (255,255,255))

    pts = np.array([[590,430],[590,380],[540,430],[540,380]], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(img, [pts], True, (0,255,255))
    cv2.line(img, [565,405], [twistJoystickTorque.x, twistJoystickTorque.y], (255,255,255))

    cv2.putText(img,text,(10,100), font, 4,(255,255,255),2,cv2.LINE_AA)

    filename = text + ".png"
    cv2.imwrite(filename,img)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
      
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
