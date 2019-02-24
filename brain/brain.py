import socket
import ConfigParser
import time
import cv2
import imutils

from picamera.array import PiRGBArray
from picamera import PiCamera

def main():
    configParser = ConfigParser.RawConfigParser()   
    configFilePath = r'config.txt'
    configParser.read(configFilePath)
    DEVICE_NO = configParser.get('DEVICE-INFO', 'deviceNo')
    UDP_PORT = configParser.get('DEVICE-INFO', 'udpPort')

    sckt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sckt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    #controlRobot(sckt)
    
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    

    #vs = VideoStream(src=0).start()
    time.sleep(1.0)

    lower = (0, 0, 0)
    upper = (255, 255, 255)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        #image = vs.read()
        blurred = cv2.GaussianBlur(image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        k = cv2.waitKey(5) & 0xFF
        if (k == 27):
            print(lower, upper)
            break
        if (k==97):
            robotLow, robotHigh, obstacleLow, obstacleHigh= configureColorRange(image, hsv)
            lowerRobot=(int(robotLow[0]), int(robotLow[1]), int(robotLow[2]))
            upperRobot=(int(robotHigh[0]), int(robotHigh[1]), int(robotHigh[2]))
            lowerObstacle=(int(obstacleLow[0]), int(obstacleLow[1]), int(obstacleLow[2]))
            upperObstacle=(int(obstacleHigh[0]), int(obstacleHigh[1]), int(obstacleHigh[2]))
            cv2.destroyAllWindows()
        #image=filterImageForRobot(image, hsv, lower, upper)
        mask = cv2.inRange(hsv.copy(), lowerRobot, upperRobot)   
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None)
        cntsRobot = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cntsRobot = imutils.grab_contours(cntsRobot)
        
        
        maskObstacle = cv2.inRange(hsv.copy(), lowerObstacle, upperObstacle)   
        maskObstacle = cv2.erode(maskObstacle, None, iterations=2)
        maskObstacle = cv2.dilate(maskObstacle, None, iterations=2)
        maskObstacle = cv2.morphologyEx(maskObstacle, cv2.MORPH_OPEN, None)
        maskObstacle = cv2.morphologyEx(maskObstacle, cv2.MORPH_CLOSE, None)
        cntsObstacle = cv2.findContours(maskObstacle.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cntsObstacle = imutils.grab_contours(cntsObstacle)
        
        if len(cntsRobot) >= 2:
            #Sort the contours by the area and check is it big enough to be a robot
            cntsRobot.sort(key=cv2.contourArea, reverse=True)
            if (cv2.contourArea(cntsRobot[0]) / cv2.contourArea(cntsRobot[1])) < 5 and cv2.contourArea(cntsRobot[0])>200:
                #cntsRobot,bbox=sort_contours(cntsRobot[:2])
                x1,y1 = getCenterOfBox(cntsRobot[0])
                x2,y2 = getCenterOfBox(cntsRobot[1])
                angle = int(math.degrees(math.atan2(y1-y2,x1-x2)))
                drawBox(cntsRobot[0], image, (x1,y1))
                drawBox(cntsRobot[1], image, (x2,y2))
                cv2.putText(image,'Angle: '+str(angle),(30, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                if cv2.contourArea(cntsRobot[0])> cv2.contourArea(cntsRobot[1]):
                    #small box is the front of the robot and
                    #left box is bigger than the right box, so its facing the right side 
                    cv2.arrowedLine(image, (x1, y1), (x2, y2), (255, 0, 0), 3)
                else: 
                    #small box is the front of the robot and
                    #right box is bigger than the left box, so its facing the left side
                    cv2.arrowedLine(image, (x2, y2), (x1, y1), (255, 0, 0), 3)

                
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
    
        #newImg=cv2.bitwise_or(mask, maskObstacle)
        cv2.imshow('Robot Detector: Press a to configure filters', image)
        rawCapture.truncate(0)

    closeSocket(sckt)
    #vs.stop()
    cv2.destroyAllWindows()

def drawBox(box, image, center):
    rect=cv2.minAreaRect(box)
    nbox=cv2.boxPoints(rect)
    nbox=np.int0(nbox)
    cv2.drawContours(image,[nbox], 0,(0,0,255),2)
    cv2.circle(image, center, 5, (0, 0, 255), -1)

def getCenterOfBox(box):
    M = cv2.moments(box)
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

def configureColorRange(image, hsvImage):
    r1 = cv2.selectROI("Select robot color", image)
    robotImg = hsvImage[int(r1[1]):int(r1[1]+r1[3]), int(r1[0]):int(r1[0]+r1[2])]
    r2 = cv2.selectROI("Select obstacle color", image)
    obstacleImg = hsvImage[int(r2[1]):int(r2[1]+r2[3]), int(r2[0]):int(r2[0]+r2[2])]

    robotLow, robotHigh, robotAvgL, robotAvgH= findRanges(robotImg)
    obstacleLow, obstacleHigh, obstacleAvgL, obstacleAvgH = findRanges(obstacleImg)
    
    #print( robotLow, robotHigh, obstacleLow, obstacleHigh)
    return robotLow, robotHigh, obstacleLow, obstacleHigh
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

    return (max(0, lowH-10), max(0, lowS-10), max(0, lowV-10)), (min(255, highH+10), min(255, highS+10), min(255, highV+10)), (max(0, tmpH-30), max(0, tmpS-30), max(0, tmpV-30)), (min(255, tmpH+30), min(255, tmpS+30), min(255, tmpV+30))


def decideAction(robotAngle, obstacleAngle):
    angle = obstacleAngle - robotAngle
    #if angle is between -15, +15 degrees, then move forward 
    if angle >= -15 and angle <= 15:
        print("forward")
        sendMessage(sckt, "255.255.255.255", 5000, 0, "Forward", 1)
    #else if angle is between 15,180 degrees then turn right
    elif angle > 15 and angle < 180:
        print("right")
        sendMessage(sckt, "255.255.255.255", 5000, 0, "Right", 1)
    #else if angle is between -15,-180 degrees then turn left
    elif angle < -15 and angle > -180:
        print("left")
        sendMessage(sckt, "255.255.255.255", 5000, 0, "Left", 1)


def sendMessage(sckt, ip, port, device, action, duration):
    message="Device:"+str(device)+", Action: "+action+", Duration: "+str(duration)
    sckt.sendto(message, (ip, port))

def closeSocket(sckt):
    sckt.close()

def signal_handler(sig, frame):
    sys.exit(0)


def controlRobot(sckt):
    while True:
        myinput = raw_input("Enter your input ->")
        if myinput == "w":
            sendMessage(sckt, "255.255.255.255", 5000, 0, "Forward", 1)
        elif myinput == "a":
            sendMessage(sckt, "255.255.255.255", 5000, 0, "Left", 1)
        elif myinput == "s":
            sendMessage(sckt, "255.255.255.255", 5000, 0, "Backward", 1)
        elif myinput == "d":
            sendMessage(sckt, "255.255.255.255", 5000, 0, "Right", 1)
        else:
            break
if __name__ == "__main__":
    main()
