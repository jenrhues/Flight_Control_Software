import socket
import time
import _thread

s = socket.socket()

s.connect(('192.168.4.1', 5000))

time.sleep(1)

def send_bno():

    heading = roll = pitch = 0.0
    w = x = y = z = 0.0

    while True:
        heading, roll, pitch = bno.euler()
        w, x, y, z = bno.quaternion()
        rssi = wlan.status('rssi')
        msg = '192.168.4.3:' + str(heading) + ',' + str(roll) + ',' + str(pitch) + ';' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(w) + ';' + str(rssi) + '\n'
        s.send(msg)
        time.sleep_ms(50)

bno_thread = _thread.start_new_thread(send_bno, ())