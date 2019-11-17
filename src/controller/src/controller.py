#!/usr/bin/env python
# rosrun enph353_utils controller.py
from __future__ import print_function

import roslib
roslib.load_manifest('enph353_utils')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = '/R1/pi_camera/image_raw'
VEL_TOPIC = '/R1/cmd_vel'

class controller:

  def __init__(self):
    self.bridge = CvBridge()
    self.vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=30)

  def send_vel(self, lin, ang):
    velocity = Twist()
    velocity.linear.x = lin
    velocity.angular.z = ang
    self.vel_pub.publish(velocity)

def main(args):
  ic = controller()
  rospy.init_node('controller', anonymous=True)
  try:
    while True:
      while rospy.get_time() == 0:
        pass

      ic.send_vel(0, 1.8)
      start = rospy.get_time()
      while rospy.get_time() - start < np.pi/3.0:
        pass
      ic.send_vel(0, 0)
      start = rospy.get_time()
      while rospy.get_time() - start < 2.0:
        pass
      ic.send_vel(0, -1.8)
      start = rospy.get_time()
      while rospy.get_time() - start < np.pi/3.0:
        pass
      ic.send_vel(0, 0)
      start = rospy.get_time()
      while rospy.get_time() - start < 2.0:
        pass
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)