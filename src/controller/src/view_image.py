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
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

LETTER_WIDTH_THRESH = 60
LETTER_HEIGHT_THRESH = 60
NUM_WIDTH_THRESH = 30
NUM_HEIGHT_THRESH = 30
BLUE_MASK = [np.array([100,100,30]), np.array([140,255,255])]
NUM_MASK = [np.array([0,0,0]), np.array([0,0,30])]
INPUT_SHAPE = (106, 160)
TEAM_ID = "Team9,UHOH"

n_white_pix = 0

sess = tf.Session()
graph = tf.get_default_graph()
set_session(sess)
model = load_model('bigbrain.h5')
model._make_predict_function()

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
    self.plate_pub = rospy.Publisher("/license_plate", String, queue_size=30)
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
    #cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    plate_lower = np.array([0,0,90], dtype=np.uint8)
    plate_upper = np.array([0,0,210], dtype=np.uint8)
    platemask = cv2.inRange(hsv, plate_lower, plate_upper)
    #platemask = cv2.blur(platemask, (3, 3))

    #road_lower = np.array([0,0,130], dtype=np.uint8)
    #road_upper = np.array([0,0,255], dtype=np.uint8)
    #roadmask = cv2.inRange(hsv, road_lower, road_upper)

    #res = cv2.bitwise_and(cv_image,cv_image, mask= mask1)

    # cv2.imshow("Robot Camera", cv_image)
    # cv2.imshow("mask", platemask)
    #cv2.imshow("res", res)

    # SUM THA WHITE PIXELS IN MASK
    n_white_pix = np.sum(platemask==255)
    
    #print(str(n_white_pix))

    if n_white_pix > 23000 and self.seen is False:
      self.seen = True
      #cv2.imwrite(os.path.join(self.PATH, "run_{}.png".format(self.count)), cv_image)
      #cv2.imwrite(os.path.join(self.PATH, "mask_{}.png".format(self.count)), platemask)
      self.platefinder(cv_image, platemask)
      self.count = self.count + 1
      print("seen :~)")

      #SEND TO CORNER FINDER

    elif n_white_pix < 12000 and self.seen is True:
      self.seen = False
      print("not seen :~(")
    
    cv2.waitKey(10)

  def to_letter(self, ind):
    return chr(ind + 48) if ind < 10 else chr(ind + 55)

  def send_plate(self, num, plate):
    msg = String()
    msg.data = TEAM_ID + "," + num + "," + plate
    self.plate_pub.publish(msg)

  def platefinder(self, rawpic, maskpic):
    global model
    global graph
    global sess
    img = maskpic.copy()
    rawimg = rawpic.copy()
    
    #img = img[:,:,np.newaxis, np.newaxis]

    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

    _, cnts, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL,
      cv2.CHAIN_APPROX_SIMPLE)

    conts = []

    for c in cnts:
        area = cv2.contourArea(c)
        peri = cv2.arcLength(c, True)
        if area > 1500 and peri < 800:
            conts.append(c)

    conts = np.array(conts)
    if conts.size > 2:
        conts = np.delete(conts, 0)

    #print(str(conts))

    for c in conts:
      cv2.drawContours(img, [c], -1, (0, 255, 0), 2)

    # smallest, biggest euclidean norm from top left corner
    botlefts = np.array([[0, 0], [0, 0]])
    botrights = np.array([[0, 0], [0, 0]])
    toplefts = np.array([[0, 0], [0, 0]])
    toprights = np.array([[0, 0], [0, 0]])

    for j in np.arange(conts.size):
        maxleftnorm = 0
        minleftnorm = 100000
        for pt in conts[j]:
            currnorm = np.linalg.norm(pt)
            if(currnorm > maxleftnorm):
                maxleftnorm = currnorm
                botrights[j] = pt
            if(currnorm < minleftnorm):
                minleftnorm = currnorm
                toplefts[j] = pt

    # smallest euclidean norm from top right corner

    for j in np.arange(conts.size):
        maxrightnorm = 0
        minrightnorm = 100000
        for pt in conts[j]:
            newpt = pt.copy()
            newpt[0][0] = 1280-newpt[0][0]
            currnorm = np.linalg.norm(newpt)
            if(currnorm > maxrightnorm):
                maxrightnorm = currnorm
                botlefts[j] = pt
            if(currnorm < minrightnorm):
                minrightnorm = currnorm
                toprights[j] = pt

    # draw tha FUCKIN CIRCLES
    '''
    for i in toprights:
        img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

    for i in botrights:
        img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

    for i in botlefts:
        img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

    for i in toplefts:
        img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)
    '''
    # perspective transform
    dst = np.array([
      [0, 0],
      [599, 0],
      [599, 297],
      [0, 297]], dtype = "float32")
    
    dst2 = np.array([
      [0, 0],
      [299, 0],
      [299, 299],
      [0, 299]], dtype = "float32")

    ptstop = np.array([toplefts[1], toprights[1], botrights[1], botlefts[1]], dtype = "float32")
    ptsplate = np.array([botlefts[1], botrights[1], toprights[0], toplefts[0]], dtype = "float32")

    M = cv2.getPerspectiveTransform(ptsplate, dst)
    N = cv2.getPerspectiveTransform(ptstop, dst2)

    plate = cv2.warpPerspective(rawimg,M,(600,298))
    top = cv2.warpPerspective(rawimg, N, (300, 300))
    '''
    cv2.imshow("Image", img)
    cv2.imshow("License Plate", plate)
    '''
    # filter characters

    platehsv = cv2.cvtColor(plate, cv2.COLOR_BGR2HSV)
    platehsv = cv2.inRange(platehsv, *BLUE_MASK)
  
    tophsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    tophsv = cv2.inRange(tophsv, *NUM_MASK)

    #find contours, boxes
    _, morecnts, _ = cv2.findContours(platehsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boundRect = [None]*len(morecnts)
    contours_poly = [None]*len(morecnts)
    for i, c in enumerate(morecnts):
      contours_poly[i] = cv2.approxPolyDP(c, 3, True)
      boundRect[i] = cv2.boundingRect(contours_poly[i])
    boundRect = sorted(boundRect, key=lambda x: x[0])

    _, morecnts, _ = cv2.findContours(tophsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boundRectTop = []
    for c in morecnts:
      poly = cv2.approxPolyDP(c, 3, True)
      boundRectTop.append(cv2.boundingRect(poly))
    boundRectTop = sorted(boundRectTop, key=lambda x: x[0], reverse=True)

    num = None
    for c in boundRectTop:
      if c[3] > NUM_HEIGHT_THRESH and c[2] > NUM_WIDTH_THRESH:
        num = cv2.resize(tophsv[c[1]:c[1]+c[3], c[0]:c[0]+c[2]], INPUT_SHAPE)
        break

    cv2.imshow("fuck", platehsv)
    if num is not None:
      cv2.imshow("Spot Number", num)

    chars = []
    for i, c in enumerate(boundRect):
      if (300 > c[0] and 300 < c[0] + c[2]) or c[3] < LETTER_HEIGHT_THRESH:
        pass
      elif c[2] > LETTER_WIDTH_THRESH and c[2] < 2 * LETTER_WIDTH_THRESH:
        chars.append(cv2.resize(platehsv[c[1]:c[1]+c[3], c[0]:c[0]+c[2]], INPUT_SHAPE))
      elif c[2] >= 2 * LETTER_WIDTH_THRESH:
        chars.append(cv2.resize(platehsv[c[1]:c[1]+c[3], c[0]:c[0]+int(c[2]/2.0)], INPUT_SHAPE))
        chars.append(cv2.resize(platehsv[c[1]:c[1]+c[3], c[0]+int(c[2]/2.0):c[0]+c[2]], INPUT_SHAPE))
    
    pred_plate = ""
    pred_num = ""
    with graph.as_default():
      set_session(sess)
      for c in chars:
        predicted = model.predict(c[:, :, np.newaxis][np.newaxis, :, :, :])
        pred_plate += self.to_letter(np.argmax(predicted[0]))
      pred = model.predict(num[:, :, np.newaxis][np.newaxis, :, :, :])
      pred_num = self.to_letter(np.argmax(pred))
    print(pred_num, pred_plate)
    self.send_plate(pred_num, pred_plate)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    # Needed to start score tracker
    ic.send_plate('0', '0')
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)