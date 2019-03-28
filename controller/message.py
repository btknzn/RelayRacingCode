import sys
sys.path.insert(0, "DifferentialDrivePathTracking/")
from main import State
class Message():
    OkMessageType = 0
    RouteMessageType = 1
    StartMessageType = 2
    GetLocationMessageType = 3
    LocationMessageType = 4
    EndMessageType = 5

    def __init__(self, type_):
        self.type = type_
    
    def __str__(self):
        return str(self.type)

    def __repr__(self):
        return str(self.type)

    @staticmethod
    #Function for parsing data and creating the specific message classes.
    def create(data):
        print("Received: " + data)
        parts = data.split(":")
        if not parts:
            return None
        if int(parts[0]) == Message.LocationMessageType:
            string = parts[1].split(",")
            a,b,c = string
            state=State(float(a), float(b), float(c))
            return LocationMessage(state)

        elif int(parts[0]) == Message.RouteMessageType:
            startStr= parts[1].split(",")
            a,b,c = startStr
            startState = State(float(a), float(b), float(c))
            targetStates = []
            for i in range(2, len(parts)):
                targetStr= parts[i].split(",")
                a,b,c = targetStr
                targetStates.append(State(float(a), float(b), float(c)))
            return RouteMessage(startState, targetStates)

        elif int(parts[0]) == Message.StartMessageType:
            return StartMessage()

        elif int(parts[0]) == Message.OkMessageType:
            return OkMessage()

        elif int(parts[0]) == Message.EndMessageType:
            return EndMessage()

        elif int(parts[0]) == Message.GetLocationMessageType:
            return GetLocationMessage()

        else:
            return None




    @staticmethod
    def createOkMessage():
        return OkMessage()


    @staticmethod
    def createGetLocationMessage():
        return GetLocationMessage()


    @staticmethod
    def createStartMessage():
        return StartMessage()


    @staticmethod
    def createEndMessage():
        return EndMessage()


    @staticmethod
    def createLocationMessage(state):
        return LocationMessage(state)


    @staticmethod
    def createRouteMessage(start, target):
        return RouteMessage(start, target)



class LocationMessage(Message):
    def __init__(self, state):
        Message.__init__(self, Message.LocationMessageType)
        self.location = state
        return
    def __str__(self):
        return super().__str__()+":"+self.location.__str__()
    def __repr__(self):
        return super().__str__()+":"+self.location.__str__()

class RouteMessage(Message):
    def __init__(self, start_, target_):
        Message.__init__(self, Message.RouteMessageType)
        self.start = start_
        self.target = target_
        return
    
    def __str__(self):
        tmpstr=super().__str__()+":"+self.start.__str__()
        for i in range(len(self.target)):
            tmpstr = tmpstr + ":" + self.target[i].__str__()
        return tmpstr

    def __repr__(self):
        tmpstr=super().__str__()+":"+self.start.__str__()
        for i in range(len(self.target)):
            tmpstr = tmpstr + ":" + self.target[i].__str__()
        return tmpstr


class StartMessage(Message):
    def __init__(self):
        Message.__init__(self, Message.StartMessageType)
        return

class OkMessage(Message):
    def __init__(self):
        Message.__init__(self, Message.OkMessageType)
        return

class EndMessage(Message):
    def __init__(self):
        Message.__init__(self, Message.EndMessageType)
        return

class GetLocationMessage(Message):
    def __init__(self):
        Message.__init__(self, Message.GetLocationMessageType)
        return