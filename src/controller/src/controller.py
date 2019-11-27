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
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = '/R1/pi_camera/image_raw'
VEL_TOPIC = '/R1/cmd_vel'

WIDTH = 1280
HEIGHT = 720

INIT = 0
INIT_FORWARD = 1
INIT_TURN = 2
INIT_FORWARD2 = 3
INIT_TURN2 = 4
ALIGN = 5
STRAIGHT = 6
PEDESTRIAN = 7
LEAVE_ZEBRA = 8
INNER_TURN = 9
INNER_STRAIGHT = 10
WAIT_FOR_TRUCK = 11
STOP = 100

SCAN_YFOLLOW = 580
SCAN_YZEBRA = [710, 715, 718]
SCAN_XZEBRA = [200, 1080]
SCAN_YPED = [435, 455, 475, 495]
SCAN_XPED = [490, 890]
SCAN_YPARKED = 516
SCAN_XPARKED = 100
RIGHT_FOLLOW_BLUE = 1020
RIGHT_FOLLOW = 960
LEFT_FOLLOW = 300
TURN_THRESH = 40
ZEBRA_THRESH = 250000
PED_THRESH = 4000
PARKEDIN_THRESH = 20000
PARKED_THRESH = 10000
PARKED_COUNT_END = 6
PARKED_WAIT_FRAMES = 14
INNER_TURN_WAIT_FRAMES = 12
PED_WAIT_FRAMES = 15
TRUCK_WAIT_FRAMES = 50
TRUCK_AREA_THRESH = 20000

WHITE_MASK = [np.array([0,0,100]), np.array([255,0,255])]
BLUE_MASK = [np.array([110,120,95]), np.array([130,255,210])]
JEAN_MASK = [np.array([95, 50, 80]), np.array([110, 200, 200])]
TRUCK_MASK1 = [np.array([0, 0, 10]), np.array([0, 0, 35])]
TRUCK_MASK2 = [np.array([0, 0, 50]), np.array([0, 0, 75])]

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

    left = -1
    if self.follow_state < INNER_TURN:
      line_mask = cv2.inRange(frame[SCAN_YFOLLOW,0:WIDTH][np.newaxis,:], *WHITE_MASK)
      for i in range(0, WIDTH):
        if line_mask[0, i] == 255 and left == -1:
          left = i
        elif left != -1:
          left = (i + left)/2
          break
    elif self.follow_state < INNER_STRAIGHT:
      line_mask = cv2.inRange(frame[SCAN_YFOLLOW,0:WIDTH/2-50][np.newaxis,:], *WHITE_MASK)
      for i in range(WIDTH/2-50-1, 0, -1):
        if line_mask[0, i] == 255 and left == -1:
          left = i
        elif left != -1:
          left = (i + left)/2
          break

    right = -1
    if self.follow_state < INNER_TURN:
      line_mask = cv2.inRange(frame[SCAN_YFOLLOW,0:WIDTH][np.newaxis,:], *WHITE_MASK)
      for i in range(WIDTH - 1, 0, -1):
        if line_mask[0, i] == 255 and right == -1:
          right = i
        elif right != -1:
          right = (i + right)/2
          break
    else:
      line_mask = cv2.inRange(frame[SCAN_YFOLLOW,WIDTH/2+50:WIDTH][np.newaxis,:], *WHITE_MASK)
      line_mask += cv2.inRange(frame[SCAN_YFOLLOW,WIDTH/2+50:WIDTH][np.newaxis,:], *BLUE_MASK)
      for i in range(0, WIDTH/2-50):
        if line_mask[0, i] == 255:
          right = i + WIDTH/2+50
          break
      right2 = -1
      line_mask = cv2.inRange(frame[SCAN_YFOLLOW+50,WIDTH/2+50:WIDTH][np.newaxis,:], *WHITE_MASK)
      line_mask += cv2.inRange(frame[SCAN_YFOLLOW+50,WIDTH/2+50:WIDTH][np.newaxis,:], *BLUE_MASK)
      for i in range(0, WIDTH/2-50):
        if line_mask[0, i] == 255:
          right2 = i + WIDTH/2+50
          break
      if right2 < right and right2 != -1:
        right = right2

    if self.follow_state == INIT:
      self.wait = 0
      self.prev_ped = 0
      self.prev_parked = 0
      self.parked_count = 0
      self.truck_wait = 0
      self.truck_prev_state = -1
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
        zebra_count += np.sum(cv2.inRange(frame[i,SCAN_XZEBRA[0]:SCAN_XZEBRA[1]][np.newaxis,:], *WHITE_MASK))
      blue_count = np.sum(cv2.inRange(frame[SCAN_YPARKED,0:SCAN_XPARKED][np.newaxis,:], *BLUE_MASK))
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
      elif np.abs(right - RIGHT_FOLLOW) > TURN_THRESH:
        self.send_vel(0, 1 if RIGHT_FOLLOW - right > 0 else -1)
      else:
        self.send_vel(1, 0)    
    elif self.follow_state == PEDESTRIAN:
      ped_count = 0
      for i in SCAN_YPED:
        ped_count += np.sum(cv2.inRange(frame[i, SCAN_XPED[0]:SCAN_XPED[1]][np.newaxis,:], *JEAN_MASK))
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
        zebra_count += np.sum(cv2.inRange(frame[i,SCAN_XZEBRA[0]:SCAN_XZEBRA[1]][np.newaxis,:], *WHITE_MASK))
      if zebra_count < ZEBRA_THRESH:
        self.wait += 1
        if self.wait > 1:
          self.wait = 0
          print('straight')
          self.follow_state = STRAIGHT
      elif np.abs(RIGHT_FOLLOW - right) > TURN_THRESH:
        self.send_vel(0, 1 if RIGHT_FOLLOW - right > 0 else -1)
      else:
        self.send_vel(1, 0)
    elif self.follow_state == INNER_TURN:
      if right < WIDTH/4 and self.wait <= 1:
        self.wait += 1
      elif right > WIDTH/4 and (self.wait > 1 and self.wait <= 3):
        self.wait += 1
      elif right < WIDTH/4 and (self.wait > 3 and self.wait <= 5):
        self.wait += 1
      elif right > WIDTH/4 and (self.wait > 5 and self.wait < INNER_TURN_WAIT_FRAMES):
        self.wait += 1
      elif self.wait >= INNER_TURN_WAIT_FRAMES:
        self.wait = 0
        print('inner straight')
        self.follow_state = INNER_STRAIGHT
      if LEFT_FOLLOW - left > TURN_THRESH:
        self.send_vel(0, 1)
      elif left > WIDTH/2:
        self.send_vel(0, -1)
      else:
        self.send_vel(1, 0)
    elif self.follow_state == INNER_STRAIGHT:
      if right > WIDTH/4:
        mask = cv2.inRange(frame[SCAN_YFOLLOW, right][np.newaxis,np.newaxis,:], *BLUE_MASK)
        xdest = RIGHT_FOLLOW_BLUE if mask[0, 0] == 255 else RIGHT_FOLLOW
        if np.abs(right - xdest) > TURN_THRESH:
          self.send_vel(0, 1 if xdest - right > 0 else -1)
        else:
          self.send_vel(1, 0)
      else:
        self.send_vel(1, 0)
    elif self.follow_state == WAIT_FOR_TRUCK:
      if self.truck_wait > TRUCK_WAIT_FRAMES:
        self.truck_wait = 0
        print('returning to state', self.truck_prev_state)
        self.follow_state = self.truck_prev_state
        self.truck_prev_state = -1
      self.truck_wait += 1
    elif self.follow_state == STOP:
      self.send_vel(0, 0)
      self.follow_state += 1

    if self.follow_state >= INNER_TURN and self.follow_state < WAIT_FOR_TRUCK:
      truck_mask = cv2.inRange(frame, *TRUCK_MASK1)
      truck_mask += cv2.inRange(frame, *TRUCK_MASK2)
      truck_mask = cv2.blur(truck_mask, (3, 3))
      _, contours, _ = cv2.findContours(truck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      max_area = 0
      for c in contours:
        con = cv2.approxPolyDP(c, 3, True)
        area = cv2.contourArea(con)
        if area > max_area:
          max_area = area
      if max_area > TRUCK_AREA_THRESH:
        print('waiting for truck')
        self.truck_prev_state = self.follow_state
        self.follow_state = WAIT_FOR_TRUCK
        self.send_vel(0, 0)

    if self.prevx != -1:
      print(self.prevy, self.prevx, frame[self.prevy, self.prevx])
      self.prevx = -1
      self.prevy = -1

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