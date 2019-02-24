import numpy as np
import cv2
import imutils
from imutils.video import VideoStream
import time

def nothing(*arg):
	pass

def createTrackbars(windowName):
    cv2.namedWindow(windowName)
    icol = (35, 47, 54, 96, 255, 255)    # Brown
    cv2.createTrackbar('lowHue', windowName, icol[0], 255, nothing)
    cv2.createTrackbar('lowSat', windowName, icol[1], 255, nothing)
    cv2.createTrackbar('lowVal', windowName, icol[2], 255, nothing)
    cv2.createTrackbar('highHue', windowName, icol[3], 255, nothing)
    cv2.createTrackbar('highSat', windowName, icol[4], 255, nothing)
    cv2.createTrackbar('highVal', windowName, icol[5], 255, nothing)
    cv2.createTrackbar('erosionIt', windowName, 1, 255, nothing)

def getTracbarValues(windowName):
    lowHue = cv2.getTrackbarPos('lowHue', windowName)
    lowSat = cv2.getTrackbarPos('lowSat', windowName)
    lowVal = cv2.getTrackbarPos('lowVal', windowName)
    highHue = cv2.getTrackbarPos('highHue', windowName)
    highSat = cv2.getTrackbarPos('highSat', windowName)
    highVal = cv2.getTrackbarPos('highVal', windowName)
    erosionIt = cv2.getTrackbarPos('erosionIt', windowName)

    colorLow = np.array([lowHue,lowSat,lowVal])
    colorHigh = np.array([highHue,highSat,highVal])
    return colorLow, colorHigh, erosionIt

vs = VideoStream(src=0).start()
time.sleep(1.0)

windowName='colorTest'
createTrackbars(windowName)
print("Press ESC to print the current value of the hsv ranges and exit")
while True:
    image = vs.read()
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    colorLow, colorHigh, erosionIt=getTracbarValues(windowName)
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cv2.imshow('colorTest', mask)
    k = cv2.waitKey(5) & 0xFF
    if (k == 27):
        print(str(colorLow)+", "+str(colorHigh))
        break


cv2.destroyAllWindows()
vs.stop()

