# import the necessary packages
import imutils
import cv2
import numpy as np
import math

import imutils
from imutils.video import VideoStream
from imutils.contours import sort_contours
import time


def drawBox(box, image, center):
    rect=cv2.minAreaRect(box)
    nbox=cv2.boxPoints(rect)
    nbox=np.int0(nbox)
    cv2.drawContours(image,[nbox], 0,(0,0,255),2)
    cv2.circle(image, center, 5, (0, 0, 255), -1)

def getCenterOfBox(box):
    M = cv2.moments(box)
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


vs = VideoStream(src=0).start()
time.sleep(1.0)
#brown box
lower=(4, 15, 63)
upper=(30, 75, 110)
#blue
lower=(0, 117, 151)
upper=(255, 255, 255)

#tenis topu
lower=(26, 55, 121)
upper=(61, 181, 255)
#yesil karton, okul kutup
lower=(43, 101, 96)
upper=(91, 255, 255)
#yesil karton, okul kutup
lower=(35, 47, 54)
upper=(96, 255, 255)


while True:
    image = vs.read()
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)   
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if len(cnts) >= 2:
        cnts.sort(key=cv2.contourArea, reverse=True)
        if (cv2.contourArea(cnts[0]) / cv2.contourArea(cnts[1])) < 5:
            #cnts,bbox=sort_contours(cnts[:2])
            x1,y1 = getCenterOfBox(cnts[0])
            x2,y2 = getCenterOfBox(cnts[1])
            angle = int(math.degrees(math.atan2(y1-y2,x1-x2)))
            drawBox(cnts[0], image, (x1,y1))
            drawBox(cnts[1], image, (x2,y2))
            cv2.putText(image,'Angle: '+str(angle),(x2+20, y2+20), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            if cv2.contourArea(cnts[0])> cv2.contourArea(cnts[1]):
                #small box is the front of the robot and
                #left box is bigger than the right box, so its facing the right side 
                cv2.arrowedLine(image, (x1, y1), (x2, y2), (255, 0, 0), 3)
            else: 
                #small box is the front of the robot and
                #right box is bigger than the left box, so its facing the left side
                cv2.arrowedLine(image, (x2, y2), (x1, y1), (255, 0, 0), 3)

    cv2.imshow('colorTest', image)

    k = cv2.waitKey(5) & 0xFF
    if (k == 27):
        break
    if (k==97):
        print(type(cnts))
        
vs.stop()
cv2.destroyAllWindows()