# boot.py -- run on boot-up

from network import WLAN
from machine import UART, I2C
import time
import socket
import pycom
import adafruit_gps
from bno055 import *

wlan = WLAN(mode=WLAN.STA, antenna=WLAN.INT_ANT)
wlan.wifi_protocol((0,0,0,1))

uart = UART(1, baudrate=9600, pins=('P19','P20'))

i2c = I2C(0, pins=('P9','P10'))
i2c.init(I2C.MASTER, baudrate=9600)

gps = adafruit_gps.GPS(uart)
gps.send_command('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command('PMTK220,500')

bno = BNO055(i2c, transpose=(1, 2, 0))

pycom.heartbeat(False)
pycom.rgbled(0x0f00f0)

s = None