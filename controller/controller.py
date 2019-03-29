import socket
import configparser
import time
import OPi.GPIO as GPIO
import os
import signal
import sys

import math
sys.path.insert(0, "DifferentialDrivePathTracking/")
from main import Controller
from message import Message

DEVICE_NO = 0
UDP_PORT = 0



class PiController(Controller):
    Init = 0
    Ready = 1
    Running = 2
    Iterating = 3

    def __init__(self, ip= '127.0.0.1', port = 5010, bsize=1024):
        self.state = self.Init
        self.TCP_IP = ip    #ip
        self.TCP_PORT = port    #port
        self.BUFFER_SIZE = bsize    #Buffer size
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.TCP_IP, self.TCP_PORT))

        self.closed = False
        
    
    def close(self):
        self.closed = True
        self.socket.close()


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

            message = Message.create(data.decode())
            if message and message.type == Message.RouteMessageType:
                self.start = message.start
                self.target = message.target
                self.state = self.Ready

                self.socket.send(Message.createOkMessage().__str__().encode())

            else:
                pass


        elif self.state == self.Ready:
            # Listen for message
            # If message recieved and this message belongs to this controller:
                # if this message is a start message
                    # Update state into Running
                    # init controller object with start and first target state
                # else do nothing

            data = self.socket.recv(self.BUFFER_SIZE)
            message = Message.create(data.decode())
            if message and message.type == Message.StartMessageType:
                self.state = self.Running
                self.goalIndex = 0
                Controller.__init__(self, self.start, self.target[self.goalIndex])
            else:
                pass

        elif self.state == self.Running:
            # if all goals achieved and there is no more target
                # then send a end message to brain
                # set state to init
            # else
                # send get location message to the brain
                # listen for message
                # If message recieved and this message belongs to this controller:
                    # if this message is a location message
                        # iterate the PID controller using the current location and the routes
                    # else do nothing, pass
                # else:
                    # update self.goal and reset errors.
                    # set state to iterating,

            if self.goalIndex == len(self.target):
                reply = Message.createEndMessage()
                self.socket.send(reply.__str__().encode())
                self.close()
            else:
                self.goal = self.target[self.goalIndex]
                self.E = 0
                self.old_e = 0

                self.state = self.Iterating


        elif self.state == self.Iterating:
            # if the goal accuired 
                # then set the goal index to next
                # set the state to running.
            # else:
                # send get location message to the brain
                # listen for message
                # If message recieved and this message belongs to this controller:
                    # if this message is a location message
                        # iterate the PID controller using the current location and the routes
                    # else
                        # do nothing, pass

            if self.isArrived():
                self.goalIndex = self.goalIndex + 1
                self.state = self.Running
            else:
                msg = Message.createGetLocationMessage()
                self.socket.send(msg.__str__().encode())
                data = self.socket.recv(self.BUFFER_SIZE)
                message = Message.create(data.decode())
                if message and message.type == Message.LocationMessageType:
                    self.current = message.location
                    v, w = self.iteratePID()
                    self.makeAction(v, w)
                    
                else:
                    pass
        else:
            pass
    def normalize(self, v, w):
        vr, vl = self.uniToDiff(v,w)
        vmax = (2*v + math.radians(180)*self.L)/(2*self.R)
        vmin= (2*v + math.radians(-180)*self.L)/(2*self.R)

        normalVr = 5*((2*(vr-vmin))/(vmax-vmin) -1)
        normalVl = 5*((2*(vl-vmin))/(vmax-vmin) -1)
        return normalVr, normalVl

    def makeAction(self, v, w):

        nvr, nvl = self.normalize(v,w)

        # Motor code to 
        #print(nvr, nvl)
        makeMove(nvr, nvl)
        return

    def signal_handler(self, sig, frame):
        self.close()
        cleanupGPIO()
        sys.exit(0)

def makeMove(vr, vl):
    # set the pwmRight and pwmLeft pins to given vr and vl voltages
    # sleep for dt seconds
    # set the pwmLeft and 
    #TODO: Needs to be implemented
    pass


def main():

    
    controller = PiController()
    signal.signal(signal.SIGINT, controller.signal_handler)
    while not controller.closed:
        controller.run()

def main2():
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