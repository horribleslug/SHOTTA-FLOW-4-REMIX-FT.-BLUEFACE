#!/usr/bin/env python
# rosrun controller controller.py

# def click_cb(event, x, y, flags, param):
#   if event == cv2.EVENT_LBUTTONDBLCLK:
#     print(x, y)
# cv2.setMouseCallback('cam', click_cb)

# cv2.circle(frame, (int(c1), SCAN_Y1), 20, (255, 0, 0), 2)
# cv2.circle(frame, (int(c2), SCAN_Y2), 20, (255, 0, 0), 2)
# cv2.circle(frame, (int(c3), SCAN_Y1), 20, (0, 255, 0), 2)
# cv2.circle(frame, (int(c4), SCAN_Y2), 20, (0, 255, 0), 2)
# if not np.isnan(c5):
#   cv2.circle(frame, (int(c5), 360), 10, (0, 0, 255), 2)
# cv2.line(frame, (640, 0), (640, 720), (0, 0, 255), 2)

# print(c1_sum, c2_sum, c3_sum, c4_sum, c5, c6, c5 - 640)

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

SCAN_Y1 = 580
SCAN_Y2 = 630
SCAN_X1 = 520
SCAN_X2 = 760

INIT = -1
INIT_FORWARD = 0
INIT_TURN = 1
INIT_FORWARD2 = 2
INIT_TURN2 = 3
ALIGN = 4
STRAIGHT = 5
STOP = 100

LINE_THRESH = 5000
TURN_THRESH = 35

class controller:

  def __init__(self):
    self.bridge = CvBridge()
    self.vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=30)
    self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback)
    self.xaxes = np.indices((1, 1280))[1]
    self.follow_state = STOP
    cv2.namedWindow('cam', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('cam', 640, 360)

  def callback(self, data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    ret, frame = cv2.threshold(frame, 200, 255, cv2.THRESH_BINARY)

    c1_sum = np.sum(frame[SCAN_Y1,0:SCAN_X1,0]) + 0.01
    c1 = np.sum((frame[SCAN_Y1,0:SCAN_X1,0])*self.xaxes[:,0:SCAN_X1])/c1_sum
    c2_sum = np.sum(frame[SCAN_Y2,0:SCAN_X1,0]) + 0.01
    c2 = np.sum((frame[SCAN_Y2,0:SCAN_X1,0])*self.xaxes[:,0:SCAN_X1])/c2_sum
    c3_sum = np.sum(frame[SCAN_Y1,SCAN_X2:1280,0]) + 0.01
    c3 = np.sum((frame[SCAN_Y1,SCAN_X2:1280,0])*self.xaxes[:,SCAN_X2:1280])/c3_sum
    c4_sum = np.sum(frame[SCAN_Y2,SCAN_X2:1280,0]) + 0.01
    c4 = np.sum((frame[SCAN_Y2,SCAN_X2:1280,0])*self.xaxes[:,SCAN_X2:1280])/c4_sum
    c5 = ((c1*SCAN_Y2 - SCAN_Y1*c2)*(c3 - c4) - (c1 - c2)*(c3*SCAN_Y2 - SCAN_Y1*c4))/((c1 - c2)*(SCAN_Y1 - SCAN_Y2) - (SCAN_Y1 - SCAN_Y2)*(c3 - c4))
    c6 = np.sum((frame[:,360,0])*np.indices((1, 720))[1])/(np.sum(frame[:,360,0]) + 0.01)

    if self.follow_state == INIT:
      self.i = 0
      self.send_vel(1, 0)
      self.follow_state = INIT_FORWARD
    elif self.follow_state == INIT_FORWARD:
      if c1_sum < LINE_THRESH and c2_sum < LINE_THRESH and c6 >= 500:
        self.send_vel(0, 1)
        self.follow_state = INIT_TURN
    elif self.follow_state == INIT_TURN:
      if c1_sum > LINE_THRESH and c2_sum > LINE_THRESH and c3_sum > LINE_THRESH and c4_sum > LINE_THRESH:
        self.send_vel(1, 0)
        self.follow_state = INIT_FORWARD2
    elif self.follow_state == INIT_FORWARD2:
      if c1_sum < LINE_THRESH and c2_sum < LINE_THRESH:
        self.send_vel(0, 1)
        self.follow_state = INIT_TURN2
    elif self.follow_state == INIT_TURN2:
      if true:
        self.send_vel(0, 0)
        self.follow_state = STOP
  


    elif self.follow_state == ALIGN:
      if c1_sum > LINE_THRESH and c2_sum > LINE_THRESH and c3_sum > LINE_THRESH and c4_sum > LINE_THRESH:
        if np.abs(625 - c5) < TURN_THRESH:
          self.send_vel(0, 0)
          self.i += 1
          if self.i > 5:
            self.i = 0
            self.follow_state = STRAIGHT
        else:
          self.send_vel(0, 1 if 625 - c5 > 0 else -1)
          self.i = 0
    elif self.follow_state == STRAIGHT:
      self.send_vel(1, 0)
      if c3 < 640 and c4 < 640 and c6 >= 480:
        self.send_vel(0, 0)
        self.follow_state = STOP
  
    cv2.imshow('cam', frame)
    cv2.waitKey(1)

  def send_vel(self, lin, ang):
    velocity = Twist()
    velocity.linear.x = lin
    velocity.angular.z = ang
    self.vel_pub.publish(velocity)

def main(args):
  ic = controller()
  rospy.init_node('controller', anonymous=True)
  try:
    ic.follow_state = INIT
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)