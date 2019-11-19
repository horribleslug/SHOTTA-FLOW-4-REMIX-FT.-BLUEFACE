#!/usr/bin/env python
# rosrun controller controller.py
from __future__ import print_function

import roslib
roslib.load_manifest('enph353_utils')
import sys
import rospy
import cv2
import numpy as np
from numpy.linalg import inv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = '/R1/pi_camera/image_raw'
VEL_TOPIC = '/R1/cmd_vel'

WIDTH = 1280
HEIGHT = 720

INIT = -1
INIT_FORWARD = 0
INIT_TURN = 1
INIT_FORWARD2 = 2
INIT_TURN2 = 3
ALIGN = 4
STRAIGHT = 5
ZEBRA = 6
PEDESTRIAN = 7
LEAVE_RED = 8
STOP = 100

SCAN_Y1 = 580
SCAN_Y2 = 630
SCAN_XGAP = 10
SCAN_YZEBRA = [710, 715, 718]
SCAN_YPED = [360, 372, 384, 396, 408, 420]
LINE_FOLLOWX = 950
LINE_THRESH = 5000
TURN_THRESH = 55
ZEBRA_THRESH = 200000
RED_THRESH = 100000
PED_THRESH = 12000
ZEBRA_WAIT_FRAMES = 10

class controller:

  def __init__(self):
    self.bridge = CvBridge()
    self.vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=30)
    self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback)
    self.xaxes = np.indices((1, WIDTH))[1]
    self.follow_state = INIT
    cv2.namedWindow('cam', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('cam', WIDTH/2, HEIGHT/2)
    def click_cb(event, x, y, flags, param):
      if event == cv2.EVENT_LBUTTONDBLCLK:
        print(x, y)
    cv2.setMouseCallback('cam', click_cb)

  def callback(self, data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    ret, frame = cv2.threshold(frame, 95, 255, cv2.THRESH_BINARY)

    c1_sum = np.sum(frame[SCAN_Y1,0:WIDTH/2-SCAN_XGAP,0]) + 0.01
    c1 = np.sum((frame[SCAN_Y1,0:WIDTH/2-SCAN_XGAP,0])*self.xaxes[:,0:WIDTH/2-SCAN_XGAP])/c1_sum
    c2_sum = np.sum(frame[SCAN_Y2,0:WIDTH/2-SCAN_XGAP,0]) + 0.01
    c2 = np.sum((frame[SCAN_Y2,0:WIDTH/2-SCAN_XGAP,0])*self.xaxes[:,0:WIDTH/2-SCAN_XGAP])/c2_sum
    c3_sum = np.sum(frame[SCAN_Y1,WIDTH/2+SCAN_XGAP:WIDTH,0]) + 0.01
    c3 = np.sum((frame[SCAN_Y1,WIDTH/2+SCAN_XGAP:WIDTH,0])*self.xaxes[:,WIDTH/2+SCAN_XGAP:WIDTH])/c3_sum
    c4_sum = np.sum(frame[SCAN_Y2,WIDTH/2+SCAN_XGAP:WIDTH,0]) + 0.01
    c4 = np.sum((frame[SCAN_Y2,WIDTH/2+SCAN_XGAP:WIDTH,0])*self.xaxes[:,WIDTH/2+SCAN_XGAP:WIDTH])/c4_sum

    if self.follow_state == INIT:
      self.wait = 0
      self.prev_ped = 0
      self.send_vel(1, 0)
      self.follow_state = INIT_FORWARD
    elif self.follow_state == INIT_FORWARD:
      if c1_sum < LINE_THRESH and c2_sum < LINE_THRESH:
        self.send_vel(0, 1)
        self.follow_state = INIT_TURN
    elif self.follow_state == INIT_TURN:
      if c1 > 200 and c2 > 200:
        self.send_vel(1, 0)
        self.follow_state = INIT_FORWARD2
    elif self.follow_state == INIT_FORWARD2:
      if c3_sum > LINE_THRESH and c3 < 800:
        self.send_vel(0, 1)
        self.follow_state = STRAIGHT
    elif self.follow_state == STRAIGHT:
      red_count = 0
      for i in SCAN_YZEBRA:
        red_count += np.sum(frame[i,200:1080,2] - frame[i,200:1080,0])
      if red_count > RED_THRESH:
        self.send_vel(1, 0)
        self.follow_state = ZEBRA
      elif np.abs(c3 - LINE_FOLLOWX) > TURN_THRESH and c3_sum > LINE_THRESH:
        self.send_vel(0, 1 if LINE_FOLLOWX - c3 > 0 else -1)
      else:
        self.send_vel(1, 0)
    elif self.follow_state == ZEBRA:
      self.send_vel(1, 0)
      zebra_count = 0
      for i in SCAN_YZEBRA:
        zebra_count += np.sum(frame[i,200:1080,0])
      if zebra_count > ZEBRA_THRESH:
        self.send_vel(0, 0)
        self.follow_state = PEDESTRIAN
    elif self.follow_state == PEDESTRIAN:
      ped_count = 0
      for i in SCAN_YPED:
        ped_count = np.sum(frame[i,440:840,0])
      if ped_count > PED_THRESH:
        self.prev_ped += 1
      elif self.prev_ped  > 0:
        if self.wait > ZEBRA_WAIT_FRAMES:
          self.follow_state = LEAVE_RED
          self.wait = 0
          self.prev_ped = 0
          self.send_vel(1, 0)
        else:
          self.wait += 1
    elif self.follow_state == LEAVE_RED:
      red_count = 0
      for i in SCAN_YZEBRA:
        red_count += np.sum(frame[i,200:1080,2] - frame[i,200:1080,0])
      if red_count > RED_THRESH * 0.75 and self.wait == 0:
        self.wait = 1
      elif red_count < RED_THRESH * 0.5 and self.wait > 0:
        self.wait += 1
      if self.wait == 2:
        self.wait = 0
        self.follow_state = STRAIGHT
    elif self.follow_state == STOP:
      self.send_vel(0, 0)
      self.follow_state += 1

  def send_vel(self, lin, ang):
    velocity = Twist()
    velocity.linear.x = lin
    velocity.angular.z = ang
    self.vel_pub.publish(velocity)

def main(args):
  ic = controller()
  rospy.init_node('controller', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)