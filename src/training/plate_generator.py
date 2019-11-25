#!/usr/bin/env python

import string
import random
from random import randint
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw

path = os.path.dirname(os.path.realpath(__file__)) + "/"

CLEAN = 4
BLUR = 200
#STRETCH = 16

#'''
def adjust_gamma(image, gamma=1.0):

   invGamma = 1.0 / gamma
   table = np.array([((i / 255.0) ** invGamma) * 255
      for i in np.arange(0, 256)]).astype("uint8")

   return cv2.LUT(image, table)
'''
for i in range(0, CLEAN):

    # Pick two random letters
    plate_alpha = ""
    for _ in range(0, 2):
        plate_alpha += (random.choice(string.ascii_uppercase))

    # Pick two random numbers
    num = randint(0, 99)
    plate_num = "{:02d}".format(num)

    # Write plate to image
    blank_plate = cv2.imread(path+'blank_plate.png')

    # Convert into a PIL image (this is so we can use the monospaced fonts)
    blank_plate_pil = Image.fromarray(blank_plate)

    # Get a drawing context
    draw = ImageDraw.Draw(blank_plate_pil)
    monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
    draw.text((48, 50),plate_alpha + " " + plate_num, (255,0,0), font=monospace)

    # Convert back to OpenCV image and save
    blank_plate = np.array(blank_plate_pil)

    # Darken it
    gamma = 4.0/randint(8, 16)
    blank_plate = adjust_gamma(blank_plate, gamma)

    # Write license plate to file
    cv2.imwrite(os.path.join(path + "pictures/", 
                                "plate_{}{}.png".format(plate_alpha, plate_num)),
                blank_plate)
'''
#'''
for i in range(0, BLUR):
    # Pick two random letters
    plate_alpha = ""
    for _ in range(0, 2):
        plate_alpha += (random.choice(string.ascii_uppercase))

    # Pick two random numbers
    num = randint(0, 99)
    plate_num = "{:02d}".format(num)

    # Write plate to image
    blank_plate = cv2.imread(path+'blank_plate.png')

    # Convert into a PIL image (this is so we can use the monospaced fonts)
    blank_plate_pil = Image.fromarray(blank_plate)

    # Get a drawing context
    draw = ImageDraw.Draw(blank_plate_pil)
    monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
    draw.text((48, 50),plate_alpha + " " + plate_num, (255,0,0), font=monospace)

    # Convert back to OpenCV image and save
    blank_plate = np.array(blank_plate_pil)

    # Darken
    gamma = 4.0/randint(8, 16)
    blank_plate = adjust_gamma(blank_plate, gamma)

    # Apply Blur
    kernsize = randint(12, 25)
    #blank_plate = cv2.blur(blank_plate,(kernsize,kernsize))
    kernel = np.ones((kernsize,kernsize),np.float32)/(kernsize**2)
    blank_plate = cv2.filter2D(blank_plate,-1,kernel)

    # Write license plate to file
    cv2.imwrite(os.path.join(path + "pictures/", 
                                "plate_{}{}.png".format(plate_alpha, plate_num)),
                blank_plate)
#'''