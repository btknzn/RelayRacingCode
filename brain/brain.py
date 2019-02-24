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

def sendMessage(sckt, ip, port, device, action, duration):
    message="Device:"+str(device)+", Action: "+action+", Duration: "+str(duration)
    sckt.sendto(message, (ip, port))


def closeSocket(sckt):
    sckt.close()


if __name__ == "__main__":
    main()
