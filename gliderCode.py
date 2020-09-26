# Description: This file contains code for the gliders' autopilot. It is still
#   under construction. Needs implementation based on the previous year's Arduino
#   code for the Aero Team.

# import libraries here #
import time  # not needed, excess
import threading
import board  # needed for i2c declarations
import busio  # needed for i2c declarations
import adafruit_bno055  # used for Magnetometer, Accelerometer, Gyroscope
import adafruit_pca9685  # used for Servo Driver Board
import adafruit_bmp3xx  # used for Precision Altimeter
import adafruit_gps  # used for GPS
import digitalio  # used for rfm95w radio
# from Adafruit_BNO055 import BNO055


# define pins here #




# declare glider variables here #
Glider = 4  # 3 for left glider, 4 for right glider
ID = "G2"   # G1 for left glider, G2 for right glider
"""
# For Left Glider
NeoGPS::Location_t otherPlace ( 279780483L , -820247268L ); //Change Lat and Lon of desired target
float tLat = 36.52589035;
float tLon = -88.91542816;

# For Right Glider
NeoGPS::Location_t otherPlace ( 279780616L , -820247573L );
float tLat = 27.97806168;
float tLon = -82.02475739;
"""
# declare radio variables here #

### Adafruit RFM95W Declaration ###
levelOffsetZ = 0.0
levelOffsetY = 0.0

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
"""
cs = digitalio.DigitalInOut(board.D5)
reset = digitalio.DigitalInOut(board.D6)
"""

# declare Adafruit modules here #
i2c = busio.I2C(board.SCL, board.SDA)

### Adafruit BNO055 declaration ###
bno = adafruit_bno055.BNO055_I2C(i2c)

### Adafruit Servo Driver declaration ###
servo = adafruit_pca9685.PCA9685(i2c)

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

Altitude = 0.0




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
"""
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

    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    if not bmp.begin():
        raise RuntimeError('Could not find a valid BMP3 sensor, check wiring!')

    # Set up oversampling and filter initialization
    bmp.temperature_oversampling()
    bmp.pressure_oversampling()
    bmp.filter_coefficient()

    i = 0
    while i < 50:
        tempAlt = bmp.altitude() * 3.281
        i = i + 1

    global startingAlt
    startingAlt = bmp.altitude() * 3.281

    global targetAlt
    if targetAlt == -1:
        targetAlt = (bmp.altitude() * 3.281) - startingAlt

    # Initialize radio
    """
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    
    while (!rf95.init())
    {
    Serial.println("LoRa radio init failed");
    while (1);
    }
    Serial.println("LoRa radio init OK!");
    
    if (!rf95.setFrequency(RF95_FREQ))
    {
    Serial.println("setFrequency failed");
    while (1);
    }
    rf95.setTxPower(23, false);
    
    for ( int i = 200; i < 400; i ++)
    {
    pwm.setPWM(1, 0, i);
    pwm.setPWM(6, 0, i);
    delay(10);
    }
    
    pwm.setPWM(1, 0, serNeu);
    pwm.setPWM(6, 0, serNeu);
    """

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
        if state == 0:
            # do nothing
            """Really do nothing?"""
        elif state == 1:
            if not dropped:
                if Glider == 4:
                    # do something
                    """
                    pwm.setPWM(1, 0, 330);
                    pwm.setPWM(6, 0, 270);
                    """
                elif Glider == 3:
                    # do something
                    """
                    pwm.setPWM(1, 0, 270);
                    pwm.setPWM(6, 0, 330);
                    """
                dropped = True
            elif dropped:
                if Altitude >= 12:
                    # PHASE 1 (Correct Heading)
                    if headingAngleError < -4 or headingAngleError > 4:
                        stabilization()     # needs arguments
                        # Stabilization(1, 1, 3, 5);
                    # PHASE 2 (Dive & Navigate)
                    elif -4 <= headingAngleError <= 4:
                        # descentAngle = atan2(Altitude, tDistance - 41) * 180 / PI;
                        stabilization()
                        # Stabilization(3, 1, 2, -descentAngle);
                # PHASE 3 (Lose speed and increase slope)
                elif 12 > Altitude > 5:
                    stabilization()
                    # Stabilization(3, 1, 1, 10);
                # PHASE 4 (Final Flare)
                elif Altitude <= 5:
                    stabilization()
                    # Stabilization(2, 1, 0, 20);
        # SUICIDE
        elif state == 2:
            # do something
            """
            pwm.setPWM(1, 0, 200);
            pwm.setPWM(6, 0, 400);
            """


# GPS function #
def GPS():
    print('GPS() function initialized')


# Radio function #
def radio():
    print('radio() function initialized')


# Stabilization function #
def stabilization():
    print('stabilization() function initialized')


# Read sensors function #
def readSensors():
    print('readSensors() functions initialized')
    print('Reading BNO055 data...')
    while True:
        """
        sensors_event_t event;
        bno.getEvent(&event);

        pitch = event.orientation.z - levelOffsetZ;
        roll = event.orientation.y - levelOffsetY;
        currentHeading = event.orientation.x - 180;    
        """
        global heading, roll, pitch
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        heading, roll, pitch = bno.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        # Print everything out.
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))

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

        """ Arduino code
        # Uncomment for barometer readings.
        Serial.print("Altitude = ");
        Serial.print(Altitude, 1);
        Serial.println(" Ft");
        # 
        
        measuredvbat = analogRead(VBATPIN);
        measuredvbat *= 2;    // we divided by 2, so multiply back
        measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
        measuredvbat /= 1024; // convert to voltage
        """



        """ Python code
        # Print system status and self test result.
        status, self_test, error = bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')

        # Print BNO055 software revision and other diagnostic data.
        sw, bl, accel, mag, gyro = bno.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
        """

# Function for previous void loop or append to main() in while True loop #


if __name__ == '__main__':
    main()
