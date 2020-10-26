from machine import I2C, Pin
from bno055 import *
import network as nw
import time

i2c = I2C(0, scl=Pin(22), sda=Pin(23), freq=9600)

bno = BNO055(i2c)

wlan = nw.WLAN(nw.STA_IF)

wlan.active(True)

nw.phy_mode(nw.STA_IF, nw.MODE_LR)

wlan.connect('HawkWorks GS', 'aero2020')

time.sleep(5)

wlan.ifconfig(('192.168.4.3', '255.255.255.0', '192.168.4.1', '8.8.8.8'))