import socket

import time

import os
import signal
import sys

from pyA20.gpio import gpio
from pyA20.gpio import port
from time import sleep
from orangepwm import *

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

    def __init__(self, ip= '127.0.0.1', port_=5000, bsize=1024):
        self.state = self.Init
        self.TCP_IP = ip    #ip
        self.TCP_PORT = port_    #port
        self.BUFFER_SIZE = bsize    #Buffer size
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.TCP_IP, self.TCP_PORT))

        self.closed = False

        self.IN1 = port.PA7
        self.IN2 = port.PA1
        self.IN3 = port.PA0
        self.IN4 = port.PA3
        self.PWMPIN1 = port.PA6
        self.PWMPIN2 = port.PA11

        gpio.init()

        gpio.setcfg(self.IN1, gpio.OUTPUT)
        gpio.setcfg(self.IN2, gpio.OUTPUT)
        gpio.setcfg(self.IN3, gpio.OUTPUT)
        gpio.setcfg(self.IN4, gpio.OUTPUT)
        gpio.setcfg(self.PWMPIN1, gpio.OUTPUT)
        gpio.setcfg(self.PWMPIN2, gpio.OUTPUT)
        
        self.pwm1 = OrangePwm(10, self.PWMPIN1)
        self.pwm2 = OrangePwm(10, self.PWMPIN2)
        self.pwm1.start(100)
        self.pwm2.start(100)


        
    
    def close(self):
        self.closed = True
        self.socket.close()
        self.pwm1.stop()
        self.pwm2.stop()
        gpio.output(self.IN1, gpio.LOW)
        gpio.output(self.IN2, gpio.LOW)
        gpio.output(self.IN3, gpio.LOW)
        gpio.output(self.IN4, gpio.LOW)


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
                #print("Goal arrived, heading to next goal")
            else:
                msg = Message.createGetLocationMessage()
                self.socket.send(msg.__str__().encode())
                data = self.socket.recv(self.BUFFER_SIZE)
                message = Message.create(data.decode())
                if message and message.type == Message.LocationMessageType:
                    self.current = message.location
                    v, w = self.iteratePID()
                    #print(v,w)
                    self.makeAction(v, w)
                    
                else:
                    pass
        else:
            pass
    def normalize(self, v, w):
        vr, vl = self.uniToDiff(v,w)
        
        vmax = (2*v + math.radians(180)*self.L)/(2*self.R)
        vmin= (2*v + math.radians(-180)*self.L)/(2*self.R)

        normalVr = 1*((2*(vr-vmin))/(vmax-vmin) -1)
        normalVl = 1*((2*(vl-vmin))/(vmax-vmin) -1)
        return normalVr, normalVl

    def makeAction(self, v, w):

        #vr, vl = self.normalize(v,w)
        vr, vl = self.uniToDiff(v,w)
        # Motor code to 
        #print(nvr, nvl)
        if(vr>=vl):
            tmpvr = vr - vl
            tmpvl = 0
        else:
            tmpvr = 0
            tmpvl = vl - vr

        # set the pwmRight and pwmLeft pins to given vr and vl voltages
        # sleep for dt seconds
        # set the pwmLeft and 
        #TODO: Needs to be implemented
        print(self.current, self.goal, vr, vl)
        
        if (vr - vl)>2:
            
            #left turn
            gpio.output(self.IN1, gpio.HIGH)
            gpio.output(self.IN2, gpio.LOW)
            pwn=self.calculatePwnValue(vr)
            self.pwm1.changeDutyCycle(pwn) 

            gpio.output(self.IN3, gpio.LOW)
            gpio.output(self.IN4, gpio.HIGH)
            pwn=self.calculatePwnValue(vl)
            self.pwm2.changeDutyCycle(pwn)


            time.sleep(self.dt*2)
            gpio.output(self.IN1, gpio.LOW)
            gpio.output(self.IN2, gpio.LOW)
            gpio.output(self.IN3, gpio.LOW)
            gpio.output(self.IN4, gpio.LOW)


            gpio.output(self.IN1, gpio.HIGH)
            gpio.output(self.IN2, gpio.LOW)
            #pwn=self.calculatePwnValue(vr)
            pwn=self.calculatePwnValue(vr)
            self.pwm1.changeDutyCycle(pwn) 

            gpio.output(self.IN3, gpio.HIGH)
            gpio.output(self.IN4, gpio.LOW)
            #pwn=self.calculatePwnValue(vl)
            pwn=self.calculatePwnValue(vl)
            self.pwm2.changeDutyCycle(pwn)

            time.sleep(self.dt*2)

        elif (vl- vr)>2:
            
            #right turn
            gpio.output(self.IN1, gpio.LOW)
            gpio.output(self.IN2, gpio.HIGH)
            pwn=self.calculatePwnValue(vr)
            self.pwm1.changeDutyCycle(pwn)

            gpio.output(self.IN3, gpio.HIGH)
            gpio.output(self.IN4, gpio.LOW)
            pwn=self.calculatePwnValue(vl)
            self.pwm2.changeDutyCycle(pwn)


            time.sleep(self.dt*2)
            gpio.output(self.IN1, gpio.LOW)
            gpio.output(self.IN2, gpio.LOW)
            gpio.output(self.IN3, gpio.LOW)
            gpio.output(self.IN4, gpio.LOW)

            gpio.output(self.IN1, gpio.HIGH)
            gpio.output(self.IN2, gpio.LOW)
            #pwn=self.calculatePwnValue(vr)
            pwn=self.calculatePwnValue(vr)
            self.pwm1.changeDutyCycle(pwn) 

            gpio.output(self.IN3, gpio.HIGH)
            gpio.output(self.IN4, gpio.LOW)
            #pwn=self.calculatePwnValue(vl)
            pwn=self.calculatePwnValue(vl)
            self.pwm2.changeDutyCycle(pwn) 

            time.sleep(self.dt*2)


            
        else:
            #forward
            
            gpio.output(self.IN1, gpio.HIGH)
            gpio.output(self.IN2, gpio.LOW)
            #pwn=self.calculatePwnValue(vr)
            pwn=self.calculatePwnValue(vr)
            self.pwm1.changeDutyCycle(pwn) 

            gpio.output(self.IN3, gpio.HIGH)
            gpio.output(self.IN4, gpio.LOW)
            #pwn=self.calculatePwnValue(vl)
            pwn=self.calculatePwnValue(vl)
            self.pwm2.changeDutyCycle(pwn) 
            
        

            time.sleep(self.dt*3)
        gpio.output(self.IN1, gpio.LOW)
        gpio.output(self.IN2, gpio.LOW)
        gpio.output(self.IN3, gpio.LOW)
        gpio.output(self.IN4, gpio.LOW)

        return

    def signal_handler(self, sig, frame):
        self.close()
        sys.exit(0)
  

    def calculatePwnValue(self, power):
        power=abs(power)
        pwn=power*5
        return pwn

def main():
    
    controller = PiController(ip='192.168.43.88')
    signal.signal(signal.SIGINT, controller.signal_handler)
    while not controller.closed:
        controller.run()


if __name__ == "__main__":
    main()