import socket
import ConfigParser
import time
import OPi.GPIO as GPIO
import os
import signal
import sys


def main():
    signal.signal(signal.SIGINT, signal_handler)

    configParser = ConfigParser.RawConfigParser()   
    configFilePath = r'config.txt'
    configParser.read(configFilePath)
    global DEVICE_NO
    DEVICE_NO = configParser.get('DEVICE-INFO', 'deviceNo')
    global UDP_PORT
    UDP_PORT = configParser.get('DEVICE-INFO', 'udpPort')
    IN1 = configParser.get('DEVICE-INFO', 'in1')
    IN2 = configParser.get('DEVICE-INFO', 'in2')
    IN3 = configParser.get('DEVICE-INFO', 'in3')
    IN4 = configParser.get('DEVICE-INFO', 'in4')

    initGPIO()
    initPin(IN1)
    initPin(IN2)
    initPin(IN3)
    initPin(IN4)
    setPinHigh(IN1)
    setPinLow(IN2)
    setPinHigh(IN3)
    setPinLow(IN4)


    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client.bind(("", 5000))
    print('Press Ctrl+C to quit')
    while True:
        data, addr = client.recvfrom(1024)
        print("Recieved data")
        parseData(data)

    GPIO.cleanup()
    client.close()

def signal_handler(sig, frame):
        cleanupGPIO()
        sys.exit(0)

def initGPIO():
    if socket.gethostname() == 'orangepizero':
        GPIO.setmode(GPIO.BOARD)

#Initializes the pins to output function
#all of the pins which are going to be used needs to be initialized.
def initPin(pinNo):
    if socket.gethostname() == 'orangepizero':
        GPIO.setup(pinNo, GPIO.OUT)


#Sets to logic high(3.3V)
def setPinHigh(pinNo):
    if socket.gethostname() == 'orangepizero':
        GPIO.output(pinNo, GPIO.HIGH)

#Sets to logic low(0V)
def setPinLow(pinNo):
    if socket.gethostname() == 'orangepizero':
        GPIO.output(pinNo,GPIO.LOW)

def cleanupGPIO():
    if socket.gethostname() == 'orangepizero':
        GPIO.cleanup()

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