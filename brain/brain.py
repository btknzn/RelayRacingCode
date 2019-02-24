import socket
import ConfigParser
import time



def main():
    configParser = ConfigParser.RawConfigParser()   
    configFilePath = r'config.txt'
    configParser.read(configFilePath)
    DEVICE_NO = configParser.get('DEVICE-INFO', 'deviceNo')
    UDP_PORT = configParser.get('DEVICE-INFO', 'udpPort')

    sckt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sckt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    #sendMessage(sckt, "255.255.255.255", 5000, 0, "Forward", 1)
    #sendMessage(sckt, "255.255.255.255", 5000, 0, "Backward", 1)
    #sendMessage(sckt, "255.255.255.255", 5000, 0, "Left", 1)
    #sendMessage(sckt, "255.255.255.255", 5000, 0, "Right", 1)
    
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
            
    closeSocket(sckt)

    vs = VideoStream(src=0).start()
    time.sleep(1.0)

    lower = (0, 0, 0)
    upper = (255, 255, 255)

    while True:
        image = vs.read()
        blurred = cv2.GaussianBlur(image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        k = cv2.waitKey(5) & 0xFF
        if (k == 27):
            print(lower, upper)
            break
        if (k==97):
            robotLow, robotHigh, obstacleLow, obstacleHigh= configureColorRange(image, hsv)
            lower=(int(robotLow[0]), int(robotLow[1]), int(robotLow[2]))
            upper=(int(robotHigh[0]), int(robotHigh[1]), int(robotHigh[2]))
            cv2.destroyAllWindows()
        #image=filterImageForRobot(image, hsv, lower, upper)
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
            if (cv2.contourArea(cnts[0]) / cv2.contourArea(cnts[1])) < 5 and cv2.contourArea(cnts[0])>200:
                #cnts,bbox=sort_contours(cnts[:2])
                x1,y1 = getCenterOfBox(cnts[0])
                x2,y2 = getCenterOfBox(cnts[1])
                angle = int(math.degrees(math.atan2(y1-y2,x1-x2)))
                drawBox(cnts[0], image, (x1,y1))
                drawBox(cnts[1], image, (x2,y2))
                cv2.putText(image,'Angle: '+str(angle),(30, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                if cv2.contourArea(cnts[0])> cv2.contourArea(cnts[1]):
                    #small box is the front of the robot and
                    #left box is bigger than the right box, so its facing the right side 
                    cv2.arrowedLine(image, (x1, y1), (x2, y2), (255, 0, 0), 3)
                else: 
                    #small box is the front of the robot and
                    #right box is bigger than the left box, so its facing the left side
                    cv2.arrowedLine(image, (x2, y2), (x1, y1), (255, 0, 0), 3)
    
        cv2.imshow('colorTest', mask)

        
    vs.stop()
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

    robotLow, robotHigh, avgL, avgH= findRanges(robotImg)
    obstacleLow, obstacleHigh, a,b = findRanges(obstacleImg)
    
    return robotLow, robotHigh, obstacleLow, obstacleHigh

def findRanges(image):
    lowH=255
    lowS=255
    lowV=255
    highH=0
    highS=0
    highV=0

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

    return (max(0, lowH-30), max(0, lowS-75), max(0, lowV-75)), (min(255, highH+15), min(255, highS+50), min(255, highV+50)), (max(0, tmpH-30), max(0, tmpS-60), max(0, tmpV-60)), (min(255, tmpH+30), min(255, tmpS+30), min(255, tmpV+60))

def sendMessage(sckt, ip, port, device, action, duration):
    message="Device:"+str(device)+", Action: "+action+", Duration: "+str(duration)
    sckt.sendto(message, (ip, port))


def closeSocket(sckt):
    sckt.close()


if __name__ == "__main__":
    main()
