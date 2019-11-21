import numpy as np
import cv2

filename = 'mask_0.png'
img = cv2.imread(filename)
filename2 = 'run_0.png'
rawimg = cv2.imread(filename2)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#'''
blurred = cv2.blur(gray, (2,2))

thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

_, cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
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
        print(str(newpt))
        currnorm = np.linalg.norm(newpt)
        if(currnorm > maxrightnorm):
            maxrightnorm = currnorm
            botlefts[j] = pt
        if(currnorm < minrightnorm):
            minrightnorm = currnorm
            toprights[j] = pt

# draw tha FUCKIN CIRCLES

for i in toprights:
    img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

for i in botrights:
    img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

for i in botlefts:
    img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

for i in toplefts:
    img = cv2.circle(img, (i[0], i[1]), 3, (0, 0, 255), 3)

# perspective transform
dst = np.array([
	[0, 0],
	[599, 0],
	[599, 297],
	[0, 297]], dtype = "float32")

#ptstop = np.float32(toplefts[0], toprights[0], botrights[0], botlefts[0])
ptsplate = np.array([botlefts[1], botrights[1], toprights[0], toplefts[0]], dtype = "float32")

M = cv2.getPerspectiveTransform(ptsplate, dst)

plate = cv2.warpPerspective(rawimg,M,(600,298))
'''
# Convert BGR to HSV
hsv = cv2.cvtColor(plate, cv2.COLOR_BGR2HSV)
    
lower_blue = np.array([100,80,0])
upper_blue = np.array([140,255,255])

# define range of blue color in HSV
#Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower_blue, upper_blue)
# Bitwise-AND mask and original image
res = cv2.bitwise_and(plate,plate, mask= mask)
'''

# show the image
cv2.imshow("Image", img)
cv2.imshow("RawImage", plate)
cv2.waitKey(0)
