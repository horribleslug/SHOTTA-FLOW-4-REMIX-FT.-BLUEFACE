#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('enph353_utils')
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = '/R1/pi_camera/image_raw'

WIDTH = 1280
HEIGHT = 720

#0, 0, 47 -> 0, 0, 76
#0, 0, 9 -> 0, 0, 35
#0, 115, 7 -> 8, 214, 74 (red)

class hsvtester:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback)
    self.mask = [50, 0, 0, 100, 255, 255]
    self.raw = None
    self.active = -1
    self.prevx = -1
    self.prevy = -1
    cv2.namedWindow('cam', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('cam', WIDTH/2, HEIGHT)
    cv2.setMouseCallback('cam', self.click_cb)

  def callback(self, data):
    try:
      self.raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def display(self):
    while True:
      frame = cv2.cvtColor(self.raw, cv2.COLOR_BGR2HSV)
      maskim = cv2.inRange(frame, np.array(self.mask[0:3]), np.array(self.mask[3:6]))
      maskim = cv2.cvtColor(maskim[:,:,np.newaxis], cv2.COLOR_GRAY2BGR)
      total = np.vstack((self.raw, maskim))

      cv2.imshow('cam', total)
      key = cv2.waitKey(1)
      if key != 255:
        print(key)

      if key == 27:
        break
      elif key >= 49 and key <= 54:
        self.active = key - 49
      elif key == 82:
        self.mask[self.active] = np.clip(self.mask[self.active] + 1, 0, 255)
        print(self.mask)
      elif key == 84:
        self.mask[self.active] = np.clip(self.mask[self.active] - 1, 0, 255)
        print(self.mask)

      if self.prevx != -1:
        print(self.prevy, self.prevx, frame[self.prevy, self.prevx])
        self.prevx = -1
        self.prevy = -1

  def click_cb(self, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
      self.prevx = x % WIDTH
      self.prevy = y % HEIGHT

def main(args):
  ic = hsvtester()
  rospy.init_node('hsv tester', anonymous=True)
  try:
    ic.display()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)