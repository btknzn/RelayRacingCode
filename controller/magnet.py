
from pyA20.gpio import gpio
from pyA20.gpio import port
from time import sleep
from orangepwm import *

self.IN = port.PA19

gpio.init()

gpio.setcfg(self.IN, gpio.OUTPUT)

magnet=False

print("Press any key to switch:")
x = input()

while x != "q":
    magnet = not magnet
    if magnet:
        gpio.output(self.IN, gpio.HIGH)
    else:
        gpio.output(self.IN, gpio.LOW)
    print("Press any key to switch:")
    x = input()