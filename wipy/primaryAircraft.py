# Program is designed to switch between a Hitec RC controller and
# autonomously driven drop software for the payload and gliders.

# Pilot takes into air on manual and may switch to manual/stabilize mode
#   when necessary. After first turn, PA may be switched to autonomous flight
#   or stay in the other two modes. Autonomous flight of the PA will not be
#   used in competition as it does not follow regulation; it is only for
#   senior design and UTM Aero team. The drop may be autonomous though.

import threading
import sys
import time  # used in GPS() function
import board  # needed for i2c declarations
import busio  # needed for i2c declarations
import adafruit_bno055  # used for Magnetometer, Accelerometer, Gyroscope
import adafruit_bmp3xx  # used for Precision Altimeter
import adafruit_gps  # used for GPS
from adafruit_servokit import ServoKit  # used for Servo Driver Board
import haversine as hs  # used for stabilization calculations
from numpy import arctan2, sin, cos, degrees  # used for stabilization calculations
import math  # used for pi and atan2() in state checks

tLoc = (0.0, -0.0)
gLoc = (0.0, 0.0)                   # default current location

# Declare Adafruit modules here #
i2c = busio.I2C(board.SCL, board.SDA)

### Adafruit BNO055 declaration ###
bno = adafruit_bno055.BNO055_I2C(i2c)

pitchOffset = 0.0
rollOffset = 0.0

### Adafruit Servo Driver declaration ###
kit = ServoKit(channels=8)

### Adafruit BMP3XX declaration ###
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

### Adafruit Ultimate GPS declaration ###
RX = board.RX
TX = board.TX

uart = busio.UART(TX, RX, baudrate=9600, timeout=30)

gps = adafruit_gps.GPS(uart, debug=False)

# GPS variables declared here #
tHeading = 0.0
tDistance = 0.0

# declare navigation variables here #
startingAlt = 0.0

pitch = 0.0
roll = 0.0
heading = 0.0

headingAngleError = 0.0

Altitude = 0.0

# checks if flight ready
flying = True       # pa6

# decides if state change
stateChange = False

# checks which state for flying
manualFly = True    # pa2
helpFly = False     # pa3
autonomous = False  # pa4

# checks which state for payload
pay = False         # pa0

# checks which state for gliders
glide = False       # pa1

# Bits for comm input
pa0 = 0     # payload release
pa1 = 0     # glider release
pa2 = 1     # manual
pa3 = 0     # helpFly
pa4 = 0     # autonomous
pa5 = 0     # stop comm
pa6 = 0     # start your engines

def GPS():
    print('GPS implementation')

def readSensors():
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    global heading, roll, pitch, rollOffset, pitchOffset
    heading, roll, pitch = bno.euler()

    heading = heading - 180
    roll = roll - rollOffset
    pitch = pitch - pitchOffset

    # Normalize to 0-360
    if heading < 0:
        heading = heading + 360
    global headingAngleError
    headingAngleError = tHeading - heading

    if headingAngleError >= 180:
        headingAngleError = headingAngleError - 360
    elif headingAngleError <= -180:
        headingAngleError = headingAngleError + 360

    # Get relative altitude
    global Altitude
    Altitude = (bmp.altitude() * 3.281) - startingAlt

def auto():
    print('Autonomous control initialized...')

def stabilize():
    print('Stabilizing with manual control...')

def manual():
    print('Manual control...')

def checkStates():
    print('Checking incoming states...')
    global pay, glide, manualFly, helpFly, autonomous
    valid = True

    # Needs to grab input as a priming read #

    while pa5 != 1 and pa6 == 1:
        # Ground decides to force drops
        if pa0 == 1:
            pay = True
        if pa1 == 1:
            glide = True

        # Ground decides flight state (manual, stabilize, autonomous)
        if pa2 == 1 and pa3 == 0 and pa4 == 0:
            manualFly = True
            helpFly = False
            autonomous = False
        elif pa2 == 1 and pa3 == 1 and pa4 == 0:
            manualFly = True
            helpFly = True
            autonomous = False
        elif pa2 == 0 and pa3 == 1 and pa4 == 1:    # need this one
            manualFly = False
            helpFly = True
            autonomous = True
        elif pa2 == 0 and pa3 == 0 and pa4 == 1:    # or this one
            manualFly = False
            helpFly = False
            autonomous = True
        else:
            print('Invalid states for flying...')
            valid = False
        if valid:
            changeState()
        else:
            valid = True
        # Needs to grab input after for new read - does not move on until new input #

def changeState():
    if manual and not helpFly:
        manual()
    elif manual and helpFly:
        stabilize()
    elif autonomous and not manual:
        auto()

if __name__ == '__main__':
    # Offset for sensor not being level on startup
    # global rollOffset, pitchOffset
    headingOffset, rollOffset, pitchOffset = bno.euler

    # Prepares bmp module for better accuracy (never uses tempAlt on purpose)
    for i in range(0, 50):
        tempAlt = bmp.altitude() * 3.281

    # global startingAlt
    startingAlt = bmp.altitude() * 3.281

    # Begin Threading
    gyroscope = threading.Thread(target=GPS, args=())
    gyroscope.start()

    mySensors = threading.Thread(target=readSensors, args=())
    mySensors.start()

    manual()
    while flying:
        checkStates()
    sys.exit()
