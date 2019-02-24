import socket
import ConfigParser
import time
import OPi.GPIO as GPIO


def main():

    GPIO.setmode(GPIO.BOARD)
    initPin(12)
    setPinHigh(12)

    
    configParser = ConfigParser.RawConfigParser()   
    configFilePath = r'config.txt'
    configParser.read(configFilePath)
    global DEVICE_NO
    DEVICE_NO = configParser.get('DEVICE-INFO', 'deviceNo')
    global UDP_PORT
    UDP_PORT = configParser.get('DEVICE-INFO', 'udpPort')


    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client.bind(("", 5000))
    while True:
        data, addr = client.recvfrom(1024)
        parseData(data)

    client.close()


#Initializes the pins to output function
#all of the pins which are going to be used needs to be initialized.
def initPin(pinNo):
    GPIO.setup(pinNo, GPIO.OUT)


#Sets to logic high(3.3V)
def setPinHigh(pinNo):
    GPIO.output(pinNo, GPIO.HIGH)

#Sets to logic low(0V)
def setPinLow(pinNo):
    GPIO.output(pinNo,GPIO.LOW)
def parseData(data):
    rawStr = data.split(",")
    devNo = rawStr[0].split(":")[1]
    action = rawStr[1].split(":")[1]
    duration = rawStr[2].split(":")[1]
    if devNo==DEVICE_NO:
        makeAction(action.strip(), int(duration.strip()))


def makeAction(action, duration):
    if action=="Forward":
        # TODO Implement the pin activation using GPIO
        time.sleep(duration)
        # TODO Implement the pin deactivation using GPIO
    elif action=="Backward":
        # TODO Implement the pin activation using GPIO
        time.sleep(duration)
        # TODO Implement the pin deactivation using GPIO
    elif action=="Left":
        # TODO Implement the pin activation using GPIO
        time.sleep(duration)
        # TODO Implement the pin deactivation using GPIO
    elif action=="Right":
        # TODO Implement the pin activation using GPIO
        time.sleep(duration)
        # TODO Implement the pin deactivation using GPIO
    else:
        print("Unknown action: %s, %s" % (action,duration))


if __name__ == "__main__":
    main()