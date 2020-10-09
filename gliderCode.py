# Description: This file contains code for the gliders' autopilot. It is still
#   under construction. Needs implementation based on the previous year's Arduino
#   code for the Aero Team.

# import libraries here #
import time  # used in GPS() function
import threading  # used for more fluid program
import board  # needed for i2c declarations
import busio  # needed for i2c declarations
import adafruit_bno055  # used for Magnetometer, Accelerometer, Gyroscope
# import adafruit_pca9685  # used for Servo Driver Board
import adafruit_bmp3xx  # used for Precision Altimeter
import adafruit_gps  # used for GPS
import digitalio  # used for rfm95w radio
from adafruit_servokit import ServoKit  # used for Servo Driver Board
import haversine as hs  # used for stabilization calculations
from numpy import arctan2, sin, cos, degrees  # used for stabilization calculations
import adafruit_rfm9x  # used for Radio
import math  # used for pi and atan2() in state checks
# from Adafruit_BNO055 import BNO055


# define pins here #

cs = digitalio.DigitalInOut(board.D5)       # Radio pin
reset = digitalio.DigitalInOut(board.D6)    # Radio pin

# declare glider variables here #
Glider = 4  # 3 for left glider, 4 for right glider
ID = "G2"   # G1 for left glider, G2 for right glider

tLoc = (27.97806168, -82.02475739)    # Right glider destination
# tLoc = (36.52589035, -88.91542816)      # Left glider destination
gLoc = (0.0, 0.0)                   # default current location

# declare radio variables here #

### Adafruit RFM95W Declaration ###
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, RADIO_FREQ_MHZ, baudrate=9500000)
rfm9x.signal_bandwidth = 250000
rfm9x.coding_rate = 5
rfm9x.spreading_factor = 7
rfm9x.enable_crc = True
rfm9x.ack_delay = 0.001
rfm9x.node = 2
rfm9x.destination = 1
counter = 0
ack_failed_counter = 0

# declare Adafruit modules here #
i2c = busio.I2C(board.SCL, board.SDA)

### Adafruit BNO055 declaration ###
bno = adafruit_bno055.BNO055_I2C(i2c)

pitchOffset = 0.0
rollOffset = 0.0

### Adafruit Servo Driver declaration ###
# servo = adafruit_pca9685.PCA9685(i2c)

kit = ServoKit(channels=8)

### Adafruit BMP3XX declaration ###
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

# declare navigation variables here #
targetAlt = -1
startingAlt = 0.0

stopComm = False

pitch = 0.0
roll = 0.0
heading = 0.0

headingAngleError = 0.0

headingCorrectionValue = 0.0
pitchCorrectionValue = 0.0
negPitchCorrectionValue = 0.0
rollCorrectionValue = 0.0

finalValue = 0.0
negFinalValue = 0.0

limitAnglePitch = 30.0
limitAngleRoll = 40.0

Altitude = 0.0

serMax = 180
serNeu = 90
serMin = 0

# declare gps variables here #
# gps_fix fix
tHeading = 0.0
tDistance = 0.0
gpsLat = 0.0
gpsLon = 0.0

### Adafruit Mini GPS PA1010 declaration ###
# gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False) # Use I2C interface

### Adafruit Ultimate GPS declaration ###
RX = board.RX
TX = board.TX

uart = busio.UART(TX, RX, baudrate=9600, timeout=30)

gps = adafruit_gps.GPS(uart, debug=False)

# declare barometer variables here #
""" (not needed?)
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
"""

# declare state variables here #  (if method is still used)
"""
0 = Not Released
1 = Released
2 = Kill
"""
state = 0
dropped = False

######### initialization of program ######### (use a main function)
def main():
    print('Start of program after declarations...')

    # Offset for sensor not being level on startup
    global rollOffset, pitchOffset
    headingOffset, rollOffset, pitchOffset = bno.euler

    # Set up oversampling and filter initialization (needed?)
    bmp.temperature_oversampling()
    bmp.pressure_oversampling()
    bmp.filter_coefficient()

    # Prepares bmp module for better accuracy (never uses tempAlt on purpose)
    for i in range(0, 50):
        tempAlt = bmp.altitude() * 3.281

    global startingAlt
    startingAlt = bmp.altitude() * 3.281

    global targetAlt
    if targetAlt == -1:
        targetAlt = (bmp.altitude() * 3.281) - startingAlt

    # Begin Threading
    myGPS = threading.Thread(target=GPS, args=())
    myGPS.start()

    mySensors = threading.Thread(target=readSensors, args=())
    mySensors.start()

    myRadio = threading.Thread(target=radio, args=())
    myRadio.start()

    # Begin main() while loop
    while True:
        print('Heading Angle Error: ' + str(headingAngleError))
        global dropped
        # Checks for non-released state
        if not state == 0:
            if state == 1:
                if not dropped:
                    if Glider == 4:
                        kit.servo[0].angle = 103
                        kit.servo[1].angle = 76
                    elif Glider == 3:
                        kit.servo[0].angle = 76
                        kit.servo[0].angle = 103
                    dropped = True
                elif dropped:
                    if Altitude >= 12:
                        # PHASE 1 (Correct Heading)
                        if headingAngleError < -4 or headingAngleError > 4:
                            stabilization(1, 1, 3, 5)
                        # PHASE 2 (Dive & Navigate)
                        elif -4 <= headingAngleError <= 4:
                            descentAngle = math.atan2(Altitude, tDistance - 41) * 180 / math.pi
                            stabilization(3, 1, 2, -descentAngle)
                    # PHASE 3 (Lose speed and increase slope)
                    elif 12 > Altitude > 5:
                        stabilization(3, 1, 1, 10)
                    # PHASE 4 (Final Flare)
                    elif Altitude <= 5:
                        stabilization(2, 1, 0, 20)
            # SUICIDE
            elif state == 2:
                kit.servo[0].angle = 45
                kit.servo[1].angle = 135


# GPS function #
def GPS():
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')
    last_print = time.monotonic()
    while True:
        gps.update()
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
            if not gps.has_fix:
                print('Waiting for fix...')
                continue
            global gpsLat, gpsLon, tDistance, gLoc, tLoc, tHeading
            gpsLat = gps.latitude
            gpsLon = gps.longitude
            gLoc = (gpsLat, gpsLon)
            tDistance = hs.haversine(gLoc, tLoc)

            # Calculates heading of glider
            dL = tLoc[1] - gLoc[1]
            X = cos(tLoc[0]) * sin(dL)
            Y = cos(gLoc[0]) * sin(tLoc[0]) - sin(gLoc[0]) * cos(tLoc[0]) * cos(dL)
            tHeading = arctan2(X, Y)
            tHeading = ((degrees(tHeading) + 360) % 360)


# Radio function #
def radio():
    print('radio() function initialized')
    # Look for a new packet: only accept if addresses to my_node
    packet = rfm9x.receive(with_ack=True, with_header=True)
    # If no packet was received during the timeout then None is returned.
    if packet is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print("Received (raw header):", [hex(x) for x in packet[0:4]])
        print("Received (raw payload): {0}".format(packet[4:]))
        print("RSSI: {0}".format(rfm9x.last_rssi))
        # send response .001 sec after any packet received, putting any lower will actually slow the communication
        time.sleep(.001)
        global counter, ack_failed_counter
        counter += 1
        # send a message to destination_node from my_node
        if not rfm9x.send_with_ack(bytes("response from node {} {}".format(rfm9x.node, counter), "UTF-8")):
            ack_failed_counter += 1
            print(" No Ack: ", counter, ack_failed_counter)


# Stabilization function #
def stabilization(pitchScalar, rollScalar, headScalar, Alpha0):
    global headingCorrectionValue, rollCorrectionValue, finalValue, negFinalValue, pitchCorrectionValue, negPitchCorrectionValue
    headingCorrectionValue = mapArduino(headingAngleError, 180, -180, serMin, serMax)
    pitchAngleCorrectionValue = pitch - Alpha0

    if pitchAngleCorrectionValue >= 0:
        pitchCorrectionValue = mapArduino(pitchAngleCorrectionValue, 0, -limitAnglePitch, serNeu, serMin)
        negPitchCorrectionValue = mapArduino(pitchAngleCorrectionValue, 0, -limitAnglePitch, serNeu, serMax)
    elif pitchAngleCorrectionValue < 0:
        pitchCorrectionValue = mapArduino(pitchAngleCorrectionValue, 0, limitAnglePitch, serNeu, serMax)
        negPitchCorrectionValue = mapArduino(pitchAngleCorrectionValue, 0, limitAnglePitch, serNeu, serMin)

    sumScalar = pitchScalar + rollScalar + headScalar
    finalValue = (pitchScalar * pitchCorrectionValue + rollScalar * rollCorrectionValue + headScalar * headingCorrectionValue) / sumScalar
    negFinalValue = (pitchScalar * negPitchCorrectionValue + rollScalar * rollCorrectionValue + headScalar * headingCorrectionValue) / sumScalar

    kit.servo[0].angle = finalValue
    kit.servo[1].angle = negFinalValue


# Read sensors function #
def readSensors():
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    global heading, roll, pitch
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

# Built-in map is not like the one in Arduino, so new function to do calculations
def mapArduino(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

if __name__ == '__main__':
    main()
