# main.py -- put your code here!

import socket
import pycom
import time
import _thread
import lib_imu as bno

pycom.heartbeat(False)

s = socket.socket()

time.sleep(1)

s.connect(('192.168.4.1', 5000))

time.sleep(1)

pycom.rgbled(0x00F000)

def send_bno():
    bno.set_mode('ndof')

    heading = roll = pitch = 0.0
    x = y = z = w = 0.0

    while True:
        heading, roll, pitch = bno.get_euler()
        x, y, z, w = bno.get_quaternion()
        msg = '192.168.4.2:' + str(heading) + ',' + str(roll) + ',' + str(pitch) + ';' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(w) + '\n'
        s.send(msg)
        time.sleep_ms(150)

bno_thread = _thread.start_new_thread(send_bno, ())

# while True:
#     s.send('Client still sees you!')
#     pycom.rgbled(0x00F000)
#     time.sleep(1)
#     data = ''
#     data = s.recv(1024)
#     pycom.rgbled(0xFF0000)
#     print(data)
