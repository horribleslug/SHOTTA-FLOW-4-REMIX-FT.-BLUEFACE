#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('enph353_utils')
import sys
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

#seen = False
n_white_pix = 0
#staticframe = np.zeros((360, 640))


class image_converter:
  

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
    self.seen = False
    self.PATH = os.path.dirname(os.path.realpath(__file__)) + "/runpics/"
    self.count = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows,cols,channels) = cv_image.shape
    #ret, frame = cv2.threshold(cv_image, 100, 255, cv2.THRESH_BINARY)
    cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of white color in HSV
    # change it according to your need !
    plate_lower = np.array([0,0,90], dtype=np.uint8)
    plate_upper = np.array([0,0,210], dtype=np.uint8)
    platemask = cv2.inRange(hsv, plate_lower, plate_upper)

    #road_lower = np.array([0,0,130], dtype=np.uint8)
    #road_upper = np.array([0,0,255], dtype=np.uint8)
    #roadmask = cv2.inRange(hsv, road_lower, road_upper)

    #res = cv2.bitwise_and(cv_image,cv_image, mask= mask1)

    cv2.imshow("Robot Camera", cv_image)
    cv2.imshow("mask", platemask)
    #cv2.imshow("res", res)

    # SUM THA WHITE PIXELS IN MASK
    n_white_pix = np.sum(platemask==255)
    
    #print(str(n_white_pix))

    if n_white_pix > 6000 and self.seen is False:
      self.seen = True
      cv2.imwrite(os.path.join(self.PATH, "run_{}.png".format(self.count)), cv_image)
      cv2.imwrite(os.path.join(self.PATH, "mask_{}.png".format(self.count)), platemask)
      self.count = self.count + 1
      print("seen :~)")

      #SEND TO CORNER FINDER

    elif n_white_pix < 3200 and self.seen is True:
      self.seen = False
      print("not seen :~(")
    
    cv2.waitKey(10)

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