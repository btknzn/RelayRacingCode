import socket

from message import Message
import sys
sys.path.insert(0, "DifferentialDrivePathTracking/")
from main import State
import math


def main():
    brain = Brain()
    while not brain.closed:
        brain.run()

class Brain():
    Init = 0
    Start = 1
    Running = 2

    def __init__(self, ip= '127.0.0.1', port = 5010, bsize=1024, totalRobotCount_=1):
        self.state = self.Init
        self.TCP_IP = ip    #ip
        self.TCP_PORT = port    #port
        self.BUFFER_SIZE = bsize    #Buffer size

        self.totalRobotCount= totalRobotCount_
        self.closed = False


        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.TCP_IP, self.TCP_PORT))
        s.listen(1)

        self.conn, self.addr = s.accept()

        print ('Connection address:', self.addr)
        pass

    def run(self):
        if self.state == self.Init:
            # Get the capture and identify the robots and objects.
            # Calculate the routes
            # Send the each routes to responsible robots.
            # Set current robot to zero
            # Set state to running

            # For now, skipping the capture and identifying process
            # These will be implemented later 
            start = State(-20.0, 15.0, math.radians(90))
            targets = [State(-20.0, 15.4, 0.0)]

            self.conn.send(Message.createRouteMessage(start, targets).__str__().encode())
            
            data = self.conn.recv(self.BUFFER_SIZE)
            message = Message.create(data.decode())
            # TODO: Identifying the current robot location is not implemented yet
            if message.type == Message.OkMessageType:
                self.robotIndex = 0
                self.state = self.Start

        elif self.state == self.Start:
            # If current robot index is over than number of robots:
                # switch state to init TODO: or finish
            # else:
                # send start message to the current robot
                # set state to running

            if self.robotIndex == self.totalRobotCount:
                self.close()
            else:
                #TODO:In sending message, later we need to implement which robot we are sending the message
                self.conn.send(Message.createStartMessage().__str__().encode())
                self.state = self.Running
            

        elif self.state == self.Running:
            # Listen the socket until received a message
            # If the message is a GetLocationMessage:
                # Then identify the current robot
                # Send a LocationMessage to the robot including the location of the robot
            # else if the message is EndMessage:
                # set current robot index to next robot index
                # set state to start
            # else do nothing, pass

            data = self.conn.recv(self.BUFFER_SIZE)
            message = Message.create(data.decode())
            # TODO: Identifying the current robot location is not implemented yet
            if message.type == Message.GetLocationMessageType:
                self.conn.send(Message.createLocationMessage(State(-20.0, 15.0, math.radians(90))).__str__().encode())

            elif message.type == Message.EndMessageType:
                self.robotIndex = self.robotIndex + 1
                self.state = self.Start

            else:
                pass

        else:
            pass

        

    def close(self):
        self.closed = True
        self.conn.close()
    

if __name__ == "__main__":
    main()