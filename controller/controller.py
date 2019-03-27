import socket
import configparser
import time
import OPi.GPIO as GPIO
import os
import signal
import sys

DEVICE_NO = 0
UDP_PORT = 0



class Controller():
    Init = 0
    Ready = 1
    Running = 2

    def __init__(self, ip= '127.0.0.1', port = 5005, bsize=1024):
        self.state = self.Init
        self.TCP_IP = ip    #ip
        self.TCP_PORT = port    #port
        self.BUFFER_SIZE = bsize    #Buffer size
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((TCP_IP, TCP_PORT))
        
        

        # Create listening(server type) tcp socket


        MESSAGE = "Hello, World!"

        self.socket.send(MESSAGE)
        data = self.socket.recv(BUFFER_SIZE)

    
    def exitRunning(self):
        self.socket.close


    def run(self):
        if self.state == self.Init:
            # Listen for message
            # If message recieved and this message belongs to this controller:
                # if this message is a route message
                    # Get the start and target states from the route message
                    # Update state into Ready
                    # Send okay message ??TODO re-think if this message is necessary 
                # else we are in wrong state, do nothing
            data = self.socket.recv(self.BUFFER_SIZE)

            message = Message.create(data)
            if message.type == Message.RouteMessage:
                self.start = message.start
                self.target = message.target
                self.state = self.Ready

                reply = Message.createOkReply()
                self.socket.send(reply)
            else:
                pass


        elif self.state == self.Ready:
            # Listen for message
            # If message recieved and this message belongs to this controller:
                # if this message is a start message
                    # Update state into Running
                # else do nothing

            data = self.socket.recv(self.BUFFER_SIZE)
            message = Message.create(data)
            if message.type == Message.StartMessage:
                self.state = self.Running
            else:
                pass

        elif self.state == self.Running:
            # if the goal accuired 
                # then send a end message to the brain
                # set state to init TODO: or finish executing the program
            # else
                # send get location message to the brain
                # listen for message
                # If message recieved and this message belongs to this controller:
                    # if this message is a location message
                        # iterate the PID controller using the current location and the routes
                    # else do nothing, pass

            if self.isArrived():
                reply = Message.createEndMessage()
                self.socket.send(reply)
                self.state = self.Init
            else:
                reply = Message.createGetLocationMessage()
                self.socket.send(reply)
                data = self.socket.recv(self.BUFFER_SIZE)
                message = Message.create(data)
                if message.type == Message.LocationMessage:
                    
        else:

            pass

def main():
    signal.signal(signal.SIGINT, signal_handler)

    configParser = ConfigParser.RawConfigParser()   
    configFilePath = r'config.txt'
    configParser.read(configFilePath)
    global DEVICE_NO
    DEVICE_NO = configParser.get('DEVICE-INFO', 'deviceNo')
    global UDP_PORT
    UDP_PORT = configParser.get('DEVICE-INFO', 'udpPort')
    
    IN1 = int(configParser.get('DEVICE-INFO', 'in1'))
    IN2 = int(configParser.get('DEVICE-INFO', 'in2'))
    IN3 = int(configParser.get('DEVICE-INFO', 'in3'))
    IN4 = int(configParser.get('DEVICE-INFO', 'in4'))

    initGPIO()
    initPin(IN1)
    initPin(IN2)
    initPin(IN3)
    initPin(IN4)


    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client.bind(("", 5000))
    print('Press Ctrl+C to quit')
    while True:
        data, addr = client.recvfrom(1024)
        print("Recieved data")
        parseData(data)

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
        IN1, IN2, IN3, IN4 = getMotorPins()
        setPinLow([IN1, IN2, IN3, IN4])
        GPIO.cleanup()

def getMotorPins():
    configParser = ConfigParser.RawConfigParser()   
    configFilePath = r'config.txt'
    configParser.read(configFilePath)
    IN1 = int(configParser.get('DEVICE-INFO', 'in1'))
    IN2 = int(configParser.get('DEVICE-INFO', 'in2'))
    IN3 = int(configParser.get('DEVICE-INFO', 'in3'))
    IN4 = int(configParser.get('DEVICE-INFO', 'in4'))
    return IN1, IN2, IN3, IN4

def parseData(data):
    rawStr = data.split(",")
    devNo = rawStr[0].split(":")[1]
    action = rawStr[1].split(":")[1]
    duration = rawStr[2].split(":")[1]
    if devNo==DEVICE_NO:
        makeAction(action.strip(), int(duration.strip()))


def makeAction(action, duration):
    IN1, IN2, IN3, IN4 = getMotorPins()
    if action=="Forward":
        setPinHigh(IN1)
        setPinLow(IN2)
        setPinHigh(IN3)
        setPinLow(IN4)

        time.sleep(duration)

        setPinLow(IN1)
        setPinLow(IN2)
        setPinLow(IN3)
        setPinLow(IN4)
        
    elif action=="Backward":
        setPinLow(IN1)
        setPinHigh(IN2)
        setPinLow(IN3)
        setPinHigh(IN4)

        time.sleep(duration)

        setPinLow(IN1)
        setPinLow(IN2)
        setPinLow(IN3)
        setPinLow(IN4)
        
    elif action=="Left":
        setPinLow(IN1)
        setPinLow(IN2)
        setPinHigh(IN3)
        setPinLow(IN4)

        time.sleep(duration)

        setPinLow(IN1)
        setPinLow(IN2)
        setPinLow(IN3)
        setPinLow(IN4)
        time.sleep(duration)

    elif action=="Right":
        setPinHigh(IN1)
        setPinLow(IN2)
        setPinLow(IN3)
        setPinLow(IN4)

        time.sleep(duration)

        setPinLow(IN1)
        setPinLow(IN2)
        setPinLow(IN3)
        setPinLow(IN4)

    else:
        print("Unknown action: %s, %s" % (action,duration))


if __name__ == "__main__":
    main()