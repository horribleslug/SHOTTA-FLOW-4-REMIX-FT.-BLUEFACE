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
PEDESTRIAN = 6
LEAVE_ZEBRA = 7
INNER_TURN = 8
INNER_STRAIGHT = 9
STOP = 100

SCAN_LINE = 580
SCAN_YZEBRA = [710, 715, 718]
SCAN_XZEBRA = [200, 1080]
SCAN_YPED = [379, 391, 403, 415]
SCAN_XPED = [490, 890]
SCAN_YBLUE = 516
SCAN_XBLUE = 100
LINE_FOLLOWX = 960
INNER_FOLLOWX = 280
TURN_THRESH = 55
ZEBRA_THRESH = 250000
PED_THRESH = 22000
PARKED_THRESH = 10000
PARKED_COUNT_END = 6
PARKED_WAIT_FRAMES = 12
PED_WAIT_FRAMES = 15
INNER_TURN_MIN_FRAMES = 50

class controller:

  def __init__(self, state=INIT):
    self.bridge = CvBridge()
    self.vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=30)
    self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback)
    self.xaxes = np.indices((1, WIDTH))[1]
    self.follow_state = state
    self.prevx = -1
    self.prevy = -1
    cv2.namedWindow('cam', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('cam', WIDTH/2, HEIGHT/2)
    cv2.setMouseCallback('cam', self.click_cb)

  def callback(self, data):
    try:
      image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    frame = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    if self.prevx != -1:
      print(self.prevy, self.prevx, frame[self.prevy, self.prevx])
      self.prevx = -1
      self.prevy = -1

    white_mask = cv2.inRange(frame, np.array([0,0,100]), np.array([255,0,255]))
    blue_mask = cv2.inRange(frame, np.array([110,120,95]), np.array([130,255,210]))

    left = -1
    for i in range(0, WIDTH):
      if white_mask[SCAN_LINE, i] == 255 and left == -1:
        left = i
      elif left != -1:
        left = (i + left)/2
        break

    right = -1
    if self.follow_state < INNER_STRAIGHT:
      for i in range(WIDTH - 1, 0, -1):
        if white_mask[SCAN_LINE, i] == 255 and right == -1:
          right = i
        elif right != -1:
          right = (i + right)/2
          break
    else:
      for i in range(int(WIDTH * 0.6), WIDTH):
        if white_mask[SCAN_LINE, i] == 255 and right == -1:
          right = i
        elif right != -1:
          right = (i + right)/2
          break

    if self.follow_state == INIT:
      self.wait = 0
      self.prev_ped = 0
      self.prev_parked = 0
      self.parked_count = 4
      self.send_vel(1, 0)
      print('forward')
      self.follow_state = INIT_FORWARD
    elif self.follow_state == INIT_FORWARD:
      if left == -1:
        self.send_vel(0, 1)
        print('turn')
        self.follow_state = INIT_TURN
    elif self.follow_state == INIT_TURN:
      if left >= 170:
        self.send_vel(1, 0)
        print('forward2')
        self.follow_state = INIT_FORWARD2
    elif self.follow_state == INIT_FORWARD2:
      if right <= 900 and right > 700:
        print('straight')
        self.follow_state = STRAIGHT
    elif self.follow_state == STRAIGHT:
      zebra_count = 0
      for i in SCAN_YZEBRA:
        zebra_count += np.sum(white_mask[i,SCAN_XZEBRA[0]:SCAN_XZEBRA[1]])
      blue_count = np.sum(blue_mask[SCAN_YBLUE,0:SCAN_XBLUE])
      if blue_count > PARKED_THRESH:
        self.prev_parked = 1
      elif self.prev_parked > PARKED_WAIT_FRAMES:
        self.prev_parked = 0
        self.parked_count += 1
        print(self.parked_count)
      elif self.prev_parked > 0:
        self.prev_parked += 1
      if self.parked_count >= PARKED_COUNT_END:
        print('inner turn')
        self.prev_parked = 0
        self.follow_state = INNER_TURN
      if zebra_count > ZEBRA_THRESH:
        self.send_vel(0, 0)
        print('ped')
        self.follow_state = PEDESTRIAN
      elif np.abs(right - LINE_FOLLOWX) > TURN_THRESH:
        self.send_vel(0, 1 if LINE_FOLLOWX - right > 0 else -1)
      else:
        self.send_vel(1, 0)    
    elif self.follow_state == PEDESTRIAN:
      ped_count = 0
      for i in SCAN_YPED:
        ped_count += np.sum(white_mask[i,SCAN_XPED[0]:SCAN_XPED[1]])
      if ped_count > PED_THRESH:
        self.prev_ped += 1
      elif self.prev_ped > 2:
        if self.wait > PED_WAIT_FRAMES:
          print('leave zebra')
          self.follow_state = LEAVE_ZEBRA
          self.wait = 0
          self.prev_ped = 0
          self.send_vel(1, 0)
        else:
          self.wait += 1
    elif self.follow_state == LEAVE_ZEBRA:
      zebra_count = 0
      for i in SCAN_YZEBRA:
        zebra_count += np.sum(white_mask[i,200:1080])
      if zebra_count < ZEBRA_THRESH:
        print('straight')
        self.follow_state = STRAIGHT
      elif np.abs(LINE_FOLLOWX - right) > TURN_THRESH:
        self.send_vel(0, 1 if LINE_FOLLOWX - right > 0 else -1)
      else:
        self.send_vel(1, 0)
    elif self.follow_state == INNER_TURN:
      if self.wait >= INNER_TURN_MIN_FRAMES:
        self.wait = 0
        print('inner straight')
        self.follow_state = INNER_STRAIGHT
      else:
        self.wait += 1
      if INNER_FOLLOWX - left > TURN_THRESH:
        self.send_vel(0, 1)
      elif left > WIDTH/2:
        self.send_vel(0, -1)
      else:
        self.send_vel(1, 0)
    elif self.follow_state == INNER_STRAIGHT:
      if right < WIDTH/6:
        self.send_vel(1, 0)
      elif np.abs(right - LINE_FOLLOWX) > TURN_THRESH:
        self.send_vel(0, 1 if LINE_FOLLOWX - right > 0 else -1)
      else:
        self.send_vel(1, 0)  
    elif self.follow_state == STOP:
      self.send_vel(0, 0)
      self.follow_state += 1

    cv2.circle(white_mask, (int(right), SCAN_LINE), 20, (255, 0, 0), 2)

    cv2.imshow('cam', white_mask)
    cv2.waitKey(1)

  def send_vel(self, lin, ang):
    velocity = Twist()
    velocity.linear.x = lin
    velocity.angular.z = ang
    self.vel_pub.publish(velocity)

  def click_cb(self, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
      self.prevx = x
      self.prevy = y

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