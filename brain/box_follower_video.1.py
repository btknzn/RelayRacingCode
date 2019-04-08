# import the necessary packages
import imutils
import cv2
import numpy as np
from scipy import stats
import math

import imutils
from imutils.video import VideoStream
from imutils.contours import sort_contours
import time



def main():
    vs = VideoStream(src=0, usePiCamera=False).start()
    time.sleep(1.0)

    upperRobot = (0, 0, 0)
    lowerRobot = (255, 255, 255)
    lowerRobot1 = lowerRobot2 = lowerRobot3 = lowerRobot4 = lowerRobot
    upperRobot1 = upperRobot2 = upperRobot3 = upperRobot4 = upperRobot
    upperObstacle = (0, 0, 0)
    lowerObstacle = (255, 255, 255)

    

    obsListx = np.array([])
    obsListy = np.array([])
    obsLastBox = None

    listR1x = np.array([])
    listR1y = np.array([])
    listR1angle = np.array([])
    lastResetCounterR1 = 0

    listR2x = np.array([])
    listR2y = np.array([])
    listR2angle = np.array([])
    lastResetCounterR2 = 0


    listR3x = np.array([])
    listR3y = np.array([])
    listR3angle = np.array([])
    lastResetCounterR3 = 0


    listR4x = np.array([])
    listR4y = np.array([])
    listR4angle = np.array([])
    lastResetCounterR4 = 0

    while True:

        image = vs.read()
        blurred2 = cv2.GaussianBlur(image.copy(), (25, 25), 0)
        blurred = cv2.blur(image.copy(),(25,25))
        hsv = cv2.cvtColor(blurred.copy(), cv2.COLOR_BGR2HSV)

        k = cv2.waitKey(5) & 0xFF
        if (k == 27):   #if key 'ESC' is pressed
            break
        if (k == 97):  #if key 'a' is pressed
            (lowerRobot1, upperRobot1), (lowerRobot2, upperRobot2), \
                (lowerRobot3, upperRobot3), (lowerRobot4, upperRobot4), \
                (lowerObstacle, upperObstacle) = configureColorRange(image, hsv)    
            #robotLow, robotHigh, obstacleLow, obstacleHigh= configureColorRange(image, hsv)
            cv2.destroyAllWindows()


        orangeImage = hsv.copy()
        robotImage1 = hsv.copy()
        robotImage2 = hsv.copy()
        robotImage3 = hsv.copy()
        robotImage4 = hsv.copy()

        cx1, cy1, angle1, filterRobot1 = triangle(lowerRobot1, upperRobot1, robotImage1, image) 
        if cx1:
            lastResetCounterR1 = 0
            if len(listR1x)==10:
                listR1x = np.delete(listR1x, 0, 0)
            if len(listR1y)==10:
                listR1y = np.delete(listR1y, 0, 0)  
            if len(listR1angle)==10:
                listR1angle = np.delete(listR1angle, 0, 0)                    
            listR1x = np.append(listR1x, cx1)                
            listR1y = np.append(listR1y, cy1)            
            listR1angle = np.append(listR1angle, angle1)
            x1 = int(np.mean(listR1x))
            y1 = int(np.mean(listR1y))
            degree1 = stats.circmean(listR1angle, low = -180, high=180)

            cv2.circle(image, (x1, y1), 5, (0, 0, 255), -1)
            cv2.putText(image,'angle= '+str(int(degree1)),(x1+10,y1+10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
        cx2, cy2, angle2, filterRobot2 = triangle(lowerRobot2, upperRobot2, robotImage2, image) 
        if cx2:
            lastResetCounterR2 = 0
            if len(listR2x)==10:
                listR2x = np.delete(listR2x, 0, 0)
            if len(listR2y)==10:
                listR2y = np.delete(listR2y, 0, 0)  
            if len(listR2angle)==10:
                listR2angle = np.delete(listR2angle, 0, 0)                    
            listR2x = np.append(listR2x, cx1)                
            listR2y = np.append(listR2y, cy1)            
            listR2angle = np.append(listR2angle, angle2)
            x2 = int(np.mean(listR2x))
            y2 = int(np.mean(listR2y))
            degree2 = stats.circmean(listR2angle, high=360)

            cv2.circle(image, (x2, y2), 5, (0, 0, 255), -1)
            cv2.putText(image,'angle= '+str(int(degree2)),(x2+10,y2+10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        #TODO missing for r3 and r4


        if lastResetCounterR1>20:
            listR1x, listR1y, listR1angle, lastResetCounterR1 = None, None, None, 0
            
            listR1x = np.array([])
            listR1y = np.array([])
            listR1angle = np.array([])
        lastResetCounterR1+=1  

        if lastResetCounterR2>20:
            listR2x, listR2y, listR2angle, lastResetCounterR2 = None, None, None, 0
            
            listR2x = np.array([])
            listR2y = np.array([])
            listR2angle = np.array([])
        lastResetCounterR2+=1  



        cntsRobotGreen, filterGreen = filterAndFindContours(lowerRobot1, upperRobot1, robotImage1)
        cntsRobotBlue, filterBlue = filterAndFindContours(lowerRobot2, upperRobot2, robotImage2)
        cntsRobotYellow, filterYellow = filterAndFindContours(lowerRobot3, upperRobot3, robotImage3)
        cntsRobotRed, filterRed = filterAndFindContours(lowerRobot4, upperRobot4, robotImage4)

        cntsObstacle, filterOrange = filterAndFindContours(lowerObstacle, upperObstacle, orangeImage)
        """
        if len(cntsRobotGreen) >= 2:
            #Sort the contours by the area and check is it big enough to be a robot
            cntsRobotGreen.sort(key=cv2.contourArea, reverse=True)
            #if (cv2.contourArea(cntsRobotGreen[0]) / cv2.contourArea(cntsRobotGreen[1])) < 10 and cv2.contourArea(cntsRobotGreen[0])>100:
            if cv2.contourArea(cntsRobotGreen[0])>50:
                cntsRobotGreen,bbox=sort_contours(cntsRobotGreen[:2])
                x1,y1 = getCenterOfBox(cntsRobotGreen[0])
                x2,y2 = getCenterOfBox(cntsRobotGreen[1])
                angle = int(math.degrees(math.atan2(y1-y2,x1-x2)))
                
                lastbox1 = cntsRobotGreen[0]
                lastbox2 = cntsRobotGreen[1]
                lastangle = angle
                lastx1, lastx2, lasty1, lasty2 = x1, x2, y1, y2
                lastResetCounter = 0

                if len(listx1)==10:
                    listx1 = np.delete(listx1, 0, 0)
                if len(listx2)==10:
                    listx2 = np.delete(listx1, 0, 0)                    
                if len(listy1)==10:
                    listy1 = np.delete(listx1, 0, 0)
                if len(listy2)==10:
                    listy2 = np.delete(listy2, 0, 0)
                listx1 = np.append(listx1, lastx1)
                listx2 = np.append(listx2, lastx2)
                listy1 = np.append(listy1, lasty1)
                listy2 = np.append(listy2, lasty2)
               

                if cv2.contourArea(cntsRobotGreen[0])> cv2.contourArea(cntsRobotGreen[1]):
                    #small box is the front of the robot and
                    #left box is bigger than the right box, so its facing the right side 
                    lastdirection = True
                else: 
                    #small box is the front of the robot and
                    #right box is bigger than the left box, so its facing the left side
                    lastdirection = False
        """
        if len(cntsObstacle) >= 1:
            for i in range(len(cntsObstacle)):
                if (cv2.contourArea(cntsObstacle[i])>50):
                    x1,y1 = getCenterOfBox(cntsObstacle[i])
                    drawBox(cntsObstacle[i], image, (x1, y1))
                    cv2.putText(image,'x='+str(x1)+', y='+str(y1),(x1+10,y1+10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
                    obsLastBox = cntsObstacle[i]
        
        
        
        """
        if len(cntsObstacle)>0:
            cntsObstacle.sort(key=cv2.contourArea, reverse=True)
            #check that there is an obstacle contour and its area is bigger than 200
            if cv2.contourArea(cntsObstacle[0])>200:

                x3,y3 = getCenterOfBox(cntsObstacle[0])
                angle2 = int(math.degrees(math.atan2(y1-y3,x1-x3)))
                cv2.putText(image,'Angle2: '+str(angle2),(30, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                cv2.arrowedLine(image, (x1, y1), (x3, y3), (255, 0, 255), 3)
                decideAction(angle, angle2)
        """

        
        #newImg=cv2.bitwise_or(mask, maskObstacle)
        cv2.imshow('Robot Detector: Press a to configure filters1', image)
        #cv2.imshow('Robot Detector: Press a to configure filters2', filterGreen)
        cv2.imshow('Robot Detector: Press a to configure filters3', filterRobot1)
        #cv2.imshow('Robot Detector: Press a to configure filters4', redImage)

    
    vs.stop()
    cv2.destroyAllWindows()

def filterAndFindContours(lower, upper, image, doMorph=True, doErode=True, doDilate=False):
    mask = cv2.inRange(image, lower, upper)   

    kernel = np.ones((5,5),np.uint8)
    if doErode:
        mask = cv2.erode(mask, kernel, iterations=3)
    if doDilate:
        mask = cv2.dilate(mask, kernel, iterations=2)
    if doMorph:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #cnts = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
        
    cnts = imutils.grab_contours(cnts)
    return cnts, mask

def drawBox(box, image, center):
    rect=cv2.minAreaRect(box)
    nbox=cv2.boxPoints(rect)
    nbox=np.int0(nbox)
    cv2.drawContours(image,[nbox], 0,(0,0,255),2)
    cv2.circle(image, center, 5, (0, 0, 255), -1)

def getCenterOfBox(box):
    M = cv2.moments(box)
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def triangle(lower, upper, image, originalImage):

    cntsTriangle, filterTriangle = filterAndFindContours(lower, upper, image)
    cntsTriangle.sort(key=cv2.contourArea, reverse=True)
    #print(cntsTriangle[0])

    if len(cntsTriangle) > 0 :
        peri = cv2.arcLength(cntsTriangle[0], True)
        approx = cv2.approxPolyDP(cntsTriangle[0], 0.04 * peri, True)
        if len(approx) == 3:
            x1, y1, x2, y2, x3, y3 = approx[0,0,0], approx[0,0,1], \
                approx[1,0,0],approx[1,0,1],approx[2,0,0], approx[2,0,1]
            # filter the image for the triangle, after that use contours for getting the A, B and C point for triangle. 
            # calculate the distances between AB, AC and BC. (A is (x1,y1), B is (x2,y2), C is (x3,y3))
            
            distAB = math.sqrt((x1-x2)**2+(y1-y2)**2)
            distAC = math.sqrt((x1-x3)**2+(y1-y3)**2)
            distBC = math.sqrt((x2-x3)**2+(y2-y3)**2)

            # find which one is the smallest, using if else blocks
            # in the if else block, make sure (x1,y1) is more left side point, (x2, y2) is the second and (x3, y3) the other point
            if (distAB < distAC and distAB < distBC):
                if x1<x2:
                    nx1, ny1, nx2, ny2, nx3, ny3 = x1, y1, x2, y2, x3, y3
                else:
                    nx1, ny1, nx2, ny2, nx3, ny3 = x2, y2, x1, y1, x3, y3

            elif (distAC < distAB and distAC < distBC):
                if x1<x3:
                    nx1, ny1, nx2, ny2, nx3, ny3 = x1, y1, x3, y3, x2, y2
                else:
                    nx1, ny1, nx2, ny2, nx3, ny3 = x3, y3, x1, y1, x2, y2

            else:
                if x2<x3:
                    nx1, ny1, nx2, ny2, nx3, ny3 = x2, y2, x3, y3, x1, y1
                else:
                    nx1, ny1, nx2, ny2, nx3, ny3 = x3, y3, x2, y2, x1, y1

            # after that use this formula to calculate the angle:
            # degree = math.degrees(math.radians(90) - math.atan2(y2-y1, x2-x1))
            degree = math.degrees(math.radians(90) + math.atan2(ny2-ny1, nx2-nx1))
            
            #print(nx1, ny1, nx2, ny2, nx3, ny3)
            # calculate the middle point, to calculate use this formula:
            # tmpx, tmpy = ((x1+x2)/2) + ((y1+y2)/2) 
            # cx, cy = tmpx + (x3 - tmpx)/3, tmpx + (x3 - tmpx)/3
            # return cx, cy, degree
            tmpx, tmpy = ((nx1+nx2)/2), ((ny1+ny2)/2) 
            cx, cy = tmpx + (nx3 - tmpx)/3, tmpy + (ny3 - tmpy)/3

            if cy > tmpy:
                degree =  degree + 180

            #cv2.circle(originalImage, (cx, cy), 5, (0, 0, 255), -1)
            #cv2.circle(originalImage, (nx1, ny1), 5, (0, 0, 255), -1)
            #cv2.circle(originalImage, (nx2, ny2), 5, (0, 0, 255), -1)
            #cv2.putText(originalImage,'angle= '+str(int(degree)),(cx+10,cy+10), 
            #            cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 1)
            
            #cv2.drawContours(originalImage, [cntsTriangle[0]], -1, (0, 255, 0), 2)
            return cx, cy, degree, filterTriangle
        else:
            return None, None, None, filterTriangle
    else :
        return None, None, None, filterTriangle


def configureColorRange(image, hsvImage):
    r1 = cv2.selectROI("Select robot1 color", image)
    robotImg1 = hsvImage[int(r1[1]):int(r1[1]+r1[3]), int(r1[0]):int(r1[0]+r1[2])]
    r2 = cv2.selectROI("Select robot2 color", image)
    robotImg2 = hsvImage[int(r2[1]):int(r2[1]+r2[3]), int(r2[0]):int(r2[0]+r2[2])]
    r3 = cv2.selectROI("Select robot3 color", image)
    robotImg3 = hsvImage[int(r3[1]):int(r3[1]+r3[3]), int(r3[0]):int(r3[0]+r3[2])]
    r4 = cv2.selectROI("Select robot4 color", image)
    robotImg4 = hsvImage[int(r4[1]):int(r4[1]+r4[3]), int(r4[0]):int(r4[0]+r4[2])]
    r5 = cv2.selectROI("Select obstacle color", image)
    obstacleImg = hsvImage[int(r5[1]):int(r5[1]+r5[3]), int(r5[0]):int(r5[0]+r5[2])]

    robot1Low, robot1High, robot1AvgL, robot1AvgH= findRanges(robotImg1)
    robot2Low, robot2High, robot2AvgL, robot2AvgH= findRanges(robotImg2)
    robot3Low, robot3High, robot3AvgL, robot3AvgH= findRanges(robotImg3)
    robot4Low, robot4High, robot4AvgL, robot4AvgH= findRanges(robotImg4)
    obstacleLow, obstacleHigh, obstacleAvgL, obstacleAvgH = findRanges(obstacleImg)
    
    robot1Low=(int(robot1Low[0]), int(robot1Low[1]), int(robot1Low[2]))
    robot1High=(int(robot1High[0]), int(robot1High[1]), int(robot1High[2]))

    robot2Low=(int(robot2Low[0]), int(robot2Low[1]), int(robot2Low[2]))
    robot2High=(int(robot2High[0]), int(robot2High[1]), int(robot2High[2]))

    robot3Low=(int(robot3Low[0]), int(robot3Low[1]), int(robot3Low[2]))
    robot3High=(int(robot3High[0]), int(robot3High[1]), int(robot3High[2]))

    robot4Low=(int(robot4Low[0]), int(robot4Low[1]), int(robot4Low[2]))
    robot4High=(int(robot4High[0]), int(robot4High[1]), int(robot4High[2]))

    obstacleLow=(int(obstacleLow[0]), int(obstacleLow[1]), int(obstacleLow[2]))
    obstacleHigh=(int(obstacleHigh[0]), int(obstacleHigh[1]), int(obstacleHigh[2]))
    
    print((robot1Low, robot1High), (robot2Low, robot2High), (robot3Low, robot3High), (robot4Low, robot4High), (obstacleLow, obstacleHigh))
    #return robotLow, robotHigh, obstacleLow, obstacleHigh
    return (robot1Low, robot1High), (robot2Low, robot2High), (robot3Low, robot3High), (robot4Low, robot4High), (obstacleLow, obstacleHigh)
    #return robotAvgL, robotAvgH, obstacleAvgL, obstacleAvgH
    
def findRanges(image):
    lowH=255
    lowS=255
    lowV=255
    highH=0
    highS=0
    highV=0

    #Tmp variables for calculating the average values, used for testing purposes only, not these using anymore
    tmpH=0
    tmpS=0
    tmpV=0
    count=0
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            count+=1
            if(image[i][j][0]<lowH):
                lowH=image[i][j][0]
            if(image[i][j][1]<lowS):
                lowS=image[i][j][1]
            if(image[i][j][2]<lowV):
                lowV=image[i][j][2]

            if(image[i][j][0]>highH):
                highH=image[i][j][0]
            if(image[i][j][1]>highS):
                highS=image[i][j][1]
            if(image[i][j][2]>highV):
                highV=image[i][j][2]
            tmpH+=image[i][j][0]
            tmpS+=image[i][j][1]
            tmpV+=image[i][j][2]
    if(count!=0):
        tmpH=tmpH/(count)
        tmpS=tmpS/(count)
        tmpV=tmpV/(count)

    return (max(0, lowH-5), max(0, lowS-15), max(0, lowV-25)), \
        (min(255, highH+5), min(255, highS+15), min(255, highV+25)), \
        (max(0, tmpH-10), max(0, tmpS-10), max(0, tmpV-20)), \
        (min(255, tmpH+10), min(255, tmpS+10), min(255, tmpV+20))

def fixAngle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def decideAction(robotAngle, obstacleAngle):
    angle= obstacleAngle- robotAngle
    #if angle is between -15, +15 degrees, then move forward 
    if angle>=-15 and angle<=15:
        print("forward")
    #else if angle is between 15,180 degrees then turn right
    elif angle>15 and angle<180:
        print("right")
    #else if angle is between -15,-180 degrees then turn left
    elif angle<-15 and angle>-180:
        print("left")


if __name__ == "__main__":
    main()
