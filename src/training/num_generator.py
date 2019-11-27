from random import randint
import cv2
import numpy as np
import os
import glob

path = os.path.dirname(os.path.realpath(__file__))
number = '0000.png'
SIZE_THRESH = 100
NUM_MASK = [np.array([0,0,0]), np.array([0,0,30])]

for name in sorted(glob.glob(path + '/numbers/1_*.png'), reverse=True):
    number = str(int(name[-8:-4]) + 1).zfill(4) + '.png'
    break
print(number)

for i in range(1, 9):
    # Create number
    s = str(i)
    parking_spot = 255 * np.ones(shape=[310, 240, 3], dtype=np.uint8)
    cv2.putText(parking_spot, s, (-15, 310), cv2.FONT_HERSHEY_PLAIN, 28, (0, 0, 0), 30, cv2.LINE_AA)    

    # Apply Blur
    kernsize = randint(12, 25)
    #blank_plate = cv2.blur(blank_plate,(kernsize,kernsize))
    kernel = np.ones((kernsize,kernsize),np.float32)/(kernsize**2)
    parking_spot = cv2.filter2D(parking_spot,-1,kernel)

    # Apply mask
    parking_spot = cv2.cvtColor(parking_spot, cv2.COLOR_BGR2HSV)
    park_mask = cv2.inRange(parking_spot, *NUM_MASK)

    _, morecnts, _ = cv2.findContours(park_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boundRect = [None]*len(morecnts)
    contours_poly = [None]*len(morecnts)

    for i, c in enumerate(morecnts):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])
    boundRect = sorted(boundRect, key=lambda x: x[0], reverse=True)
    for c in boundRect:
        if c[3] > SIZE_THRESH and c[3] < 300:
            park_mask = cv2.resize(park_mask[c[1]:c[1]+c[3], c[0]:c[0]+c[2]], (106, 160))
            break

    cv2.imwrite(path + '/numbers/' + s + '_' + number, park_mask)
