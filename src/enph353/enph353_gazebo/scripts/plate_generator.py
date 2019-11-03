#!/usr/bin/env python

import pyqrcode
import string
import random
from random import randint
import cv2
import numpy as np
import csv
import os

path = os.path.dirname(os.path.realpath(__file__)) + "/"
texture_path = '../media/materials/textures/'

with open(path + "plates.csv", 'w') as plates_file:
    csvwriter = csv.writer(plates_file)

    for i in range(0, 16):

        # Pick two random letters
        plate_alpha = ""
        for _ in range(0, 2):
            plate_alpha += (random.choice(string.ascii_uppercase))
        num = randint(0, 99)

        # Pick two random numbers
        plate_num = "{:02d}".format(num)

        # Save plate to file
        csvwriter.writerow([plate_alpha+plate_num])

        # Write plate to image
        blank_plate = cv2.imread(path+'blank_plate.png')
        cv2.putText(blank_plate,
                    plate_alpha + " " + plate_num, (45, 360),
                    cv2.FONT_HERSHEY_PLAIN, 11, (255, 0, 0), 7, cv2.LINE_AA)

        # Create QR code image
        spot_name = "P" + str(i)
        qr = pyqrcode.create(spot_name+"_" + plate_alpha + plate_num)
        qrname = path + "QRCode_" + str(i) + ".png"
        qr.png(qrname, scale=20)
        QR_img = cv2.imread(qrname)
        QR_img = cv2.resize(QR_img, (600, 600), interpolation=cv2.INTER_AREA)

        # Create parking spot label
        s = "P" + str(i)
        parking_spot = 255 * np.ones(shape=[600, 600, 3], dtype=np.uint8)
        cv2.putText(parking_spot, s, (30, 450), cv2.FONT_HERSHEY_PLAIN, 28,
                    (0, 0, 0), 30, cv2.LINE_AA)
        spot_w_plate = np.concatenate((parking_spot, blank_plate), axis=0)

        # Merge labelled or unlabelled images and save
        labelled = np.concatenate((QR_img, spot_w_plate), axis=0)
        unlabelled = np.concatenate((255 * np.ones(shape=[600, 600, 3],
                                    dtype=np.uint8), spot_w_plate), axis=0)
        cv2.imwrite(os.path.join(path + texture_path + "labelled/",
                                 "plate_" + str(i) + ".png"), labelled)
        cv2.imwrite(os.path.join(path+texture_path+"unlabelled/",
                                 "plate_" + str(i) + ".png"), unlabelled)
