# IMU LIBRARY
# This is a library for the BNO055 orientation sensor
import pycom
import struct
import time
from machine import I2C

# configure the I2C bus: use default PIN assignments (SDA = P09, SCL = P10)
i2c = I2C(0, pins=('P19','P20'))  # configure the I2C bus
i2c.init(I2C.MASTER, baudrate=9600)  # init as a master baudrate = freq di lettura del dato

# I2C addresses
ADDRESS_B = 0x29  # This is the alternative I2C address; 0101000b
DEFAULT_ADDRESS = 0x28  # This is the default I2C address of the BNO055 device; 0101001b
BNO055_ID = 0xA0

# Page id register definition
PAGE_ID_ADDR = 0x07  # Read: Number of currently selected page; Write: Change page, 0x00 or 0x01

# Registrer Descriprion
"""The entire communication with the device is performed by reading from and writing to register. 
Regoster have width of 8 bits. 
There are several registers which are either completely or partially  marked as 'reserved'. 
It is recommended not to use register at all which are completely marked as 'reserved'. 
The register map is separated into two logical pages, 
Page 1 contains sensor specific configuration data and Page 0 contais all other cofiguration 
parameters and output data.
"""
# PAGE 0 REGISTER DEFINITION
CHIP_ID_ADDR = 0X00  # BNO055 CHIP ID, chip identification chip, read only fixed value 0x0A
ACC_ID_ADDR = 0X01  # ACC CHIP ID, chip identification accelerometer device, read only fixed value oxFB
MAG_ID_ADDR = 0x02  # MAG CHiP ID, chip id of the magnetometer device, read only fixed value 0x32
GYR_ID_ADDR = 0x03  # GYRO CHiP ID, chip id of the gyroscope device, read only fixed value 0x0F
SW_REV_LSB_ADDR = 0X04  # Lower byte of SW Revision ID, read only fixed value depending on SW revision programmed on microcontroller
SW_REV_MSB_ADDR = 0X05  # Upper byte of SW Revision ID, read only fixed value depending on SW revision programmed on microcontroller
BL_REV_ID_ADDR = 0x06  # Bootloader version, identifies the version of the bootloader in the microcontroller, read only

# Acceleration Data Registers
"""In fusion mode the fusion algorithm outpt offset compensated acceleration data for each axis X/Y/Z,
the output data can be read from the appropriate ACC_DATA_<axis>_LSB and ACC_DATA_<axis>_MSB register.
"""
ACC_DATA_X_LSB = 0X08  # Lower byte of X axis Acceleration data, read only
ACC_DATA_X_MSB = 0X09  # Upper byte of X axis Acceleration data, read only
ACC_DATA_y_LSB = 0X0A  # Lower byte of Y axis Acceleration data, read only
ACC_DATA_Y_MSB = 0X0B  # Upper byte of Y axis Acceleration data, read only
ACC_DATA_Z_LSB = 0X0C  # Lower byte of Z axis Acceleration data, read only
ACC_DATA_Z_MSB = 0X0D  # Upper byte of Z axis Acceleration data, read only

# Magnetometer Data Registers
"""In fusion mode the fusion algorithm output offset compensated magnetic field streght data for each 
axis X/Y/Z, the output data can be read from the appropriate MAG_DATA_<axis>_LSB 
and MAG_DATA_<axis>_MSB register. 
"""
MAG_DATA_X_LSB = 0X0E  # Lower byte of X axis Magnetometer data, read only
MAG_DATA_X_MSB = 0X0F  # Upper byte of X axis Magnetometer data, read only
MAG_DATA_Y_LSB = 0X10  # Lower byte of Y axis Magnetometer data, read only
MAG_DATA_Y_MSB = 0X11  # Upper byte of Y axis Magnetometer data, read only
MAG_DATA_Z_LSB = 0X12  # Lower byte of Z axis Magnetometer data, read only
MAG_DATA_Z_MSB = 0X13  # Upper byte of Z axis Magnetometer data, read only

# Gyro Data Registers
"""In fusion mode the fusion algorithm output offset compensated angular velocity data for each axis X/Y/Z, 
the output data can be read from the appropriate GYR_DATA_<axis>_LSB and GYR_DATA_<axis>_MSB register.
"""
GYR_DATA_X_LSB = 0X14  # Lower byte of X axis Gyroscope data, read only
GYR_DATA_X_MSB = 0X15  # Upper byte of X axis Gyroscope data, read only
GYR_DATA_Y_LSB = 0X16  # Lower byte of Y axis Gyroscope data, read only
GYR_DATA_Y_MSB = 0X17  # Upper byte of Y axis Gyroscope data, read only
GYR_DATA_Z_LSB = 0X18  # Lower byte of Z axis Gyroscope data, read only
GYR_DATA_Z_MSB = 0X19  # Upper byte of Z axis Gyroscope data, read only

# Orientation: orientation output only available in fusion operation modes.

# Euler Angles Data Registers
"""The fusion algorithm output offset and tilt compensated orientation data in Euler angles format 
for each DOF Heading/Roll/Pitch, the output data can be read from the appropriate EUL<dof>_LSB 
and EUL_<dof>_MSB registers.
"""
EUL_DATA_X_LSB = 0X1A  # Lower byte of heading data, read only
EUL_DATA_X_MSB = 0X1B  # Upper byte of heading data, read only
EUL_DATA_Y_LSB = 0X1C  # Lower byte of roll data, read only
EUL_DATA_Y_MSB = 0X1D  # Upper byte of roll data, read only
EUL_DATA_Z_LSB = 0X1E  # Lower byte of pitch data, read only
EUL_DATA_Z_MSB = 0X1F  # Upper byte of pitch data, read only

# Quaternion Data Registers
"""The fusion algorithm output offset and tilt compensated orientation data in Euler angles format 
for each DOF Heading/Roll/Pitch, the output data can be read from the appropriate EUL<dof>_LSB 
and EUL_<dof>_MSB registers.
"""
QUA_DATA_W_LSB = 0X20  # Lower byte of w axis quaternion data, read only
QUA_DATA_W_MSB = 0X21  # Upper byte of w axis quaternion data, read only
QUA_DATA_X_LSB = 0X22  # Lower byte of X axis quaternion data, read only
QUA_DATA_X_MSB = 0X23  # Upper byte of X axis quaternion data, read only
QUA_DATA_Y_LSB = 0X24  # Lower byte of Y axis quaternion data, read only
QUA_DATA_Y_MSB = 0X25  # Upper byte of Y axis quaternion data, read only
QUA_DATA_Z_LSB = 0X26  # Lower byte of Z axis quaternion data, read only
QUA_DATA_Z_MSB = 0X27  # Upper byte of Z axis quaternion data, read only

# Linear Acceleration Data Registers
"""Linear acceleration output only available in fusion operating modes. 
The fusion algorithm output linear acceleration data for each axis x/y/z, 
the output data can be read from the appropriate LIA_DATA_<axis>_LSB and LIA_DATA_<axis>_MSB registers.
"""
LIA_DATA_X_LSB = 0X28  # Lower byte of X axis Linear Acceleration data, read only
LIA_DATA_X_MSB = 0X29  # Upper byte of X axis Linear Acceleration data, read only
LIA_DATA_Y_LSB = 0X2A  # Lower byte of Y axis Linear Acceleration data, read only
LIA_DATA_Y_MSB = 0X2B  # Upper byte of Y axis Linear Acceleration data, read only
LIA_DATA_Z_LSB = 0X2C  # Lower byte of Z axis Linear Acceleration data, read only
LIA_DATA_Z_MSB = 0X2D  # Upper byte of Z axis Linear Acceleration data, read only

# Gravity Vector Data Registers
"""Gravity Vector output only available in fusion operating modes. 
The fusion algorithm output gravity vector data for each axis x/y/z, the output data
can be read from the appropriate GRV_DATA_<axis>_LSB and GRV_DATA_<axis>_MSB registers.
"""
GRV_DATA_X_LSB = 0X2E  # Lower byte of X axis gravity vector data, read only
GRV_DATA_X_MSB = 0X2F  # Upper byte of X axis gravity vector data, read only
GRV_DATA_Y_LSB = 0X30  # Lower byte of Y axis gravity vector data, read only
GRV_DATA_Y_MSB = 0X31  # Upper byte of Y axis gravity vector data, read only
GRV_DATA_Z_LSB = 0X32  # Lower byte of Z axis gravity vector data, read only
GRV_DATA_Z_MSB = 0X33  # Upper byte of Z axis gravity vector data, read only

# Temperature Data Register
TEMP = 0x34  # Read only, data output source can be selectednby updating the TEMP_SOURCE register
TEMP_SOURCE = 0X40  # The temperature source can be selected by writing to this register.
TEMP_SOURCE_ACC = 0x00  # xxxxxx00b, the source is the accelerometer
TEMP_SOURCE_GYR = 0x01  # xxxxxx01b, the source is the gyroscope

# Status Registers
CALIB_STAT = 0X35  # The register can be read to see the calibration status of the gyro, the acc and the magn
SELFTEST_RESULT = 0X36
INTR_STAT = 0X37
SYS_CLK_STAT = 0X38
SYS_STAT = 0X39
SYS_ERR = 0X3A

# Unit selection Register
UNIT = 0x3B
""""The measurement units for the various data outputs can be configured by writing to the UNIT_SEL register
    -bit 0 select acceleration units, xxxxxxx0b m/s^2, xxxxxxx1b mg
    -bit 1 select angular rate units, xxxxxx0xb Dps, xxxxxx1xb RPS
    -bit 2 select euler units, xxxxx0xxb Degrees, xxxxx1xxb Radians
    -bit 3 reserved
    -bit 4 select temperature units, xxx0xxxxb Celsius, xxx1xxxx Fhahrenheit
    -bit 5-6 reserved
    -bit 7 select orentation mode. 
The data output format can be selectedby writing to the UNIT_SEL register, this allows user to switch 
between the orentation definition described by Windows and Android operating system.
"""

# Mode Registers
OPR_MODE = 0x3D  # Register to configure the operation mode. Bits <7:4> reserved
PWR_MODE = 0x3E  # Register to configure the power modes. Bits <7:2> reserved
SYS_TRIGGER = 0x3F

# Axis Remap Registers
"""The axis  of the device can be re-configured to the new reference axis.
    -Bit <7:3> are reserved.
    -Bit <2> remapped X axis sign.
    -Bit <1> remapped Y axis sign. 
    -Bit <0> remapped Z axis sign. 
Value 0 = positive, value 1 = negative.
"""
AXIS_MAP_CONFIG = 0x41  # bit<7:6> are reserved, 00b X-Axis bit<0:1>, 01b Y-Axis bit<2:3>, 10b Z-Axis bit<4:5>.
AXIS_MAP_SIGN = 0x42  # Remapped axis signed.

# Axis Remap Values
AXIS_DEFAULT = 0x24  # The default value is: Xaxis = X, Y axis =m Y, Z axis = Z; xx100100b
REMAP_X = 0x00
REMAP_Y = 0x01
REMAP_Z = 0x02
REMAP_POSITIVE = 0x00
REMAP_NEGATIVE = 0x01

# Accelerometer Offset registers
"""There are 6 bytes required to configre the accelerometer offset.
Configuration will take place only when the user writes the last byte.
"""
ACCEL_OFFSET_X_LSB = 0X55
ACCEL_OFFSET_X_MSB = 0X56
ACCEL_OFFSET_Y_LSB = 0X57
ACCEL_OFFSET_Y_MSB = 0X58
ACCEL_OFFSET_Z_LSB = 0X59
ACCEL_OFFSET_Z_MSB = 0X5A

# Magnetometer Offset registers
"""There are 6 bytes required to configre the magnetometer offset.
Configuration will take place only when the user writes the last byte.
"""
MAG_OFFSET_X_LSB = 0X5B
MAG_OFFSET_X_MSB = 0X5C
MAG_OFFSET_Y_LSB = 0X5D
MAG_OFFSET_Y_MSB = 0X5E
MAG_OFFSET_Z_LSB = 0X5F
MAG_OFFSET_Z_MSB = 0X60

# Gyroscope Offset registers
"""There are 6 bytes required to configure the gyroscope offset.
Configuration will take place only when the user writes the last byte.
"""
GYRO_OFFSET_X_LSB = 0X61
GYRO_OFFSET_X_MSB = 0X62
GYRO_OFFSET_Y_LSB = 0X63
GYRO_OFFSET_Y_MSB = 0X64
GYRO_OFFSET_Z_LSB = 0X65
GYRO_OFFSET_Z_MSB = 0X66

# Radius registers
""" The radius of accelerometer, magnetometer and gyroscope can be configured in the following registers.
There are 4 bytes (2 bytes for each accelerometer and magnetometer) to configure the radius.
Configuration will take place only when user writes to the last byte.
"""
ACCEL_RADIUS_LSB = 0X67
ACCEL_RADIUS_MSB = 0X68
MAG_RADIUS_LSB = 0X69
MAG_RADIUS_MSB = 0X6A

# Operation Mode
# The default operation mode after power-on (or RESET) is CONFIGMODE, this mode is used to configure BNO.
# CONFFIGMODE is the only mode in which all the writable register map entries can be changed.
# The operating mode can be selected by writing to the OPR_MODE register.
CONFIGMODE = 0X00
ACCONLY = 0X01  # Non-Fusion Mode
MAGONLY = 0X02  # Non-Fusion Mode
GYRONLY = 0X03  # Non-Fusion Mode
ACCMAG = 0X04  # Non-Fusion Mode
ACCGYRO = 0X05  # Non-Fusion Mode
MAGGYRO = 0X06  # Non-Fusion Mode
AMG = 0X07  # Non-Fusion Mode
IMUPLUS = 0X08  # Fusion Mode
COMPASS = 0X09  # Fusion Mode
M4G = 0X0A  # Fusion Mode
NDOF_FMC_OFF = 0X0B  # Fusion Mode
NDOF = 0x0C  # Operation mode that use accel, gyro and mag. Reg Value xxxx1100b. There are some different types.

# Power Modes
# The power mode can be selected by writing to the PWR_MODE register.
# As default at start-up the BNO055 will run in Normal mode.
# If no activity is detected for a configurable duration (5 seconds), the BNO055 enters the low power mode.
# In normal mode all sensors required fot the selected operating mode are always switched ON.
NormalMode = 0x00  # It is the default mode. Reg Value xxxxxx00b
LowPowerMode = 0x01  # In this mode only accelerometer is active. Reg Value xxxxxx01b
SuspendMode = 0x02  # THe system is paused and all the sensor and the microcontroller are put into sleep mode.

# PAGE 1 REGISTER DEFINITION
# Sensor configuration
ACC_CONFIG = 0x08  # The accelerometer configuration can be changed by writing to the ACC_CONFIG register
MAG_CONFIG = 0x09  # The magnetometer configuration can be changed by writing to the MAG_CONFIG register
# The gyroscope configuration can be changed by writing to the GYR_CONFIG register
GYR_CONFIG_0 = 0x0A  # Bit <5:3> GYR-Bandwidth, bit <2:0> GYR-Range
GYR_CONFIG_1 = 0x0B  # Bit <2:0> GYR-power-mode

# VALUES
ACC_DATA = ACC_DATA_X_LSB
MAG_DATA = MAG_DATA_X_LSB
GYR_DATA = GYR_DATA_X_LSB
EUL_DATA = EUL_DATA_X_LSB
LIA_DATA = LIA_DATA_X_LSB
GRV_DATA = GRV_DATA_X_LSB
QUA_DATA = QUA_DATA_W_LSB

# Operation Modes
_modes = {
    "acc": ACCONLY,
    "magn": MAGONLY,
    "gyro": GYRONLY,
    "accmag": ACCMAG,
    "accgyro": ACCGYRO,
    "maggyro": MAGGYRO,
    "amg": AMG,
    "imu": IMUPLUS,
    "comp": COMPASS,
    "m4g": M4G,
    "ndof_off": NDOF_FMC_OFF,
    "ndof": NDOF
}

# Power Modes
_power = {
    "normal": NormalMode,
    "low": LowPowerMode,
    "suspend": SuspendMode
}


def set_configmode():
    # make sure we are on page 0
    i2c.writeto_mem(DEFAULT_ADDRESS, PAGE_ID_ADDR, 0)
    # Enter config mode
    i2c.writeto_mem(DEFAULT_ADDRESS, OPR_MODE, CONFIGMODE)
    time.sleep(0.03)  # Delay for 30 milliseconds


# Enter operation mode with ' '
def set_mode(mode):
    if mode not in _modes:
        raise ValueError
    set_configmode()
    time.sleep(0.03)  # Delay for 30 milliseconds
    i2c.writeto_mem(DEFAULT_ADDRESS, OPR_MODE, _modes[mode])
    time.sleep(0.02)  # Delay for 20 milliseconds


# configure different power mode
def set_power(power):
    if power not in _power:
        raise ValueError
    i2c.writeto_mem(DEFAULT_ADDRESS, OPR_MODE, CONFIGMODE)
    time.sleep(0.03)  # Delay for 30 milliseconds
    i2c.writeto_mem(DEFAULT_ADDRESS, PWR_MODE, _power[power])


# configure default units
def set_units():
    i2c.writeto_mem(DEFAULT_ADDRESS, UNIT, 0)


def set_acc():
    i2c.writeto_mem(DEFAULT_ADDRESS, PAGE_ID_ADDR, 0x01)
    i2c.writeto_mem(DEFAULT_ADDRESS, OPR_MODE, ACC_CONFIG)
    time.sleep(0.03)
    i2c.writeto_mem(DEFAULT_ADDRESS, ACCEL_OFFSET_X_LSB, 0x00)

def set_gyro():
    i2c.writeto_mem(DEFAULT_ADDRESS, PAGE_ID_ADDR, 0x01)
    i2c.writeto_mem(DEFAULT_ADDRESS, OPR_MODE, GYR_CONFIG_1)
    time.sleep(0.03)
    i2c.writeto_mem(DEFAULT_ADDRESS, GYRO_OFFSET_X_LSB, 0x00)

# Read Vector
def read_vector(address, count=3):
    # Read count number of 16-bit signed values starting from the provided
    # address. Returns a tuple of the values that were read.
    data = i2c.readfrom_mem(DEFAULT_ADDRESS, address, count * 2)
    result = [0] * count
    for i in range(count):
        result[i] = ((data[i * 2 + 1] << 8) | data[i * 2]) & 0xFFFF
        if result[i] > 32767:
            result[i] -= 65536
    return result


def read_vector_int(address, count=3):
    data = i2c.readfrom_mem(DEFAULT_ADDRESS, address, count * 2)
    # result = [0]*count
    # for i in range(count):
    #     result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF
    return data


def calib_stat(address=CALIB_STAT, count=4):
    data = i2c.readfrom_mem(DEFAULT_ADDRESS, address, count * 2)
    # result = [0]*count
    # for i in range(count):
    #     result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF
    return data


def get_calibration():
    """Return the sensor's calibration data and return it as an array of
    22 bytes. Can be saved and then reloaded with the set_calibration function
    to quickly calibrate from a previously calculated set of calibration data.
    """
    # Switch to configuration mode
    set_configmode()
    # Read the 22 bytes of calibration data and convert it to a list (from
    # a bytearray) so it's more easily serialized should the caller want to
    # store it.
    cal_data = list(i2c.readfrom_mem(DEFAULT_ADDRESS, ACCEL_OFFSET_X_LSB, 22))
    # Go back to normal operation mode.
    set_mode('ndof_off')  # CAMBIARE MODALITà
    return cal_data


def set_calibration(data):
    """Set the sensor's calibration data using a list of 22 bytes that
    represent the sensor offsets and calibration data.  This data should be
    a value that was previously retrieved with get_calibration (and then
    perhaps persisted to disk or other location until needed again).
    """
    # Check that 22 bytes were passed in with calibration data.
    if data is None or len(data) != 22:
        raise ValueError('Expected a list of 22 bytes for calibration data.')
    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    set_configmode()
    # Set the 22 bytes of calibration data.
    i2c.writeto_mem(DEFAULT_ADDRESS, ACCEL_OFFSET_X_LSB, data)
    # Go back to normal operation mode.
    set_mode('ndof_off')


# Read accelerometer
def get_accelerometer():
    # Return the current accelerometer reading as a tuple of X, Y, Z values in maters/second^2
    # print("funzione get accelerometer")
    x, y, z = read_vector(ACC_DATA)
    return (x / 100.0, y / 100.0, z / 100.0)


# Read magnetometer
def get_magnetometer():
    # Return the current magnetometer reading as a tuple of X, Y, Z values in micro-Tesla
    x, y, z = read_vector(MAG_DATA)
    return x / 16.0, y / 16.0, z / 16.0


# Read gyroscope
def get_gyroscope():
    # Return the current gyroscope (angular velocity) reading as a tuple of X, Y, Z values in degrees per second
    print("funzione get gyroscope")
    x, y, z = read_vector(GYR_DATA)
    return (x / 900.0, y / 900.0, z / 900.0)


# Read euler
def get_euler():
    # Return the current absolute orientation as a tuple of heading, roll, and pitch euler angles in degrees
    # print("funzione get euler")
    heading, roll, pitch = read_vector(EUL_DATA)
    return (heading / 16.0, roll / 16.0, pitch / 16.0)


def get_euler_int():
    data = i2c.readfrom_mem(DEFAULT_ADDRESS, EUL_DATA, 3 * 2)
    return data


# Read linear acceleration
def get_linear_acceleration():
    # Return the current linear acceleration (acceleration from movement, not from gravity)
    # reading as a tuple of X, Y, Z values in meters/second^2
    print("funzione get linear acceleration")
    x, y, z = read_vector(LIA_DATA)
    return (x / 100.0, y / 100.0, z / 100.0)


# Read gravity
def get_gravity():
    # Return the current gravity acceleration reading as a tuple of X, Y, Z values in meters/second^2
    print("funzione get gravity")
    x, y, z = read_vector(GRV_DATA)
    return (x / 100.0, y / 100.0, z / 100.0)


# Read quaternion
def get_quaternion():
    # Return the current orientation as a tuple of X, Y, Z, W quaternion values
    # print("funzione get quaternion")
    w, x, y, z = read_vector(QUA_DATA, 4)
    # Scale values
    scale = (1.0 / (1 << 14))
    return  x * scale, y * scale, z * scale,w * scale


def get_status():
    # Return the current orientation as a tuple of X, Y, Z, W quaternion values
    # print("funzione get quaternion")
    cal_status = i2c.readfrom_mem(DEFAULT_ADDRESS, CALIB_STAT, 1)

    # Return the results as a tuple of all 3 values.
    return cal_status


# Read temperature
def get_temperature():
    # Retrieves the current temperature in Celtius.
    print("funzione get temperature")
    temp = i2c.readfrom_mem(DEFAULT_ADDRESS, TEMP, 1)[0]
    if temp > 127:
        return temp - 256
    else:
        return temp


def reset():
    set_configmode()
    # Set the 22 bytes of calibration data.
    i2c.writeto_mem(DEFAULT_ADDRESS, SYS_TRIGGER, 0x20)
    time.sleep(1)
    # Go back to normal operation mode.
    set_mode('ndof')
