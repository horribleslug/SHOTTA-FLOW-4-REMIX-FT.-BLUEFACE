#!/usr/bin/env python

import string
import random
from random import randint
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw

path = os.path.dirname(os.path.realpath(__file__)) + "/"

CLEAN = 16
BLUR = 16
#STRETCH = 16

#'''
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

    # Write license plate to file
    cv2.imwrite(os.path.join(path + "pictures/", 
                                "plate_{}{}.png".format(plate_alpha, plate_num)),
                blank_plate)
#'''
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

    # Apply Blur
    kernsize = randint(10, 20)
    #kernsize = 10
    blank_plate = cv2.blur(blank_plate,(kernsize,kernsize))

    # Write license plate to file
    cv2.imwrite(os.path.join(path + "pictures/", 
                                "plate_{}{}.png".format(plate_alpha, plate_num)),
                blank_plate)
#'''