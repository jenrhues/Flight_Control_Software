# main.py -- put your code here!

import pycom
import time
import _thread


heading = roll = pitch = 0.0
x = y = z = w = 0.0
lat = lng = gps_quality = 0.0
alt = 0.0

def data_link():
    global s
    while True:
        try:
            while not wlan.isconnected() or s is None:
                while not wlan.isconnected():
                    s = None
                    #print("attempting reconnect...")
                    pycom.rgbled(0xf000f0)
                    wlan.connect('HawkWorks GS', auth=(WLAN.WPA2, 'aero2020'))
                    wlan.ifconfig(config=('192.168.4.2', '255.255.255.0', '192.168.4.1', '8.8.8.8'))
                    time.sleep(1)

                while s is None:
                    #print("attempting socket reconnect...")
                    s = socket.socket()
                    s.connect(('192.168.4.1', 5000))
                    data_thread = _thread.start_new_thread(send_data, ())
                    pycom.rgbled(0x00F000)
        except OSError:
            print("Reconnect failed! Retrying...")
            s = None

def send_data():

    global s
    global heading
    global roll
    global pitch
    global x
    global y
    global z
    global w
    global lat
    global lng
    global gps_quality
    global alt

    while s != None:
        try:
            rssi = wlan.joined_ap_info()[3]
            msg = '192.168.4.2:' + str(heading) + ',' + str(roll) + ',' + str(pitch) + ';' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(w) + ';' + str(rssi) + ';' + str(lat) + ',' + str(lng) + ',' + str(gps_quality) + ';' + str(alt) + '\n'
            s.send(msg)
            time.sleep_ms(50)
        except OSError:
            print("Connection Lost! Reconnecting...")
            s = None
            break

def get_sensors():
    #bno.set_mode('ndof')

    global heading
    global roll
    global pitch
    global x
    global y
    global z
    global w
    global alt

    while True:
        heading, roll, pitch = bno.euler()
        w, x, y, z = bno.quaternion()

        alt = bmp.altitude
        
        time.sleep_ms(75)

def get_gps():

    global lat
    global lng
    global gps_quality
    
    while True:
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property).
        gps.update()
        
        if gps.has_fix:            
            # We have a fix! (gps.has_fix is true)
            
            lat = gps.latitude
            lng = gps.longitude
            gps_quality = gps.fix_quality

    time.sleep_ms(250)


sensor_thread = _thread.start_new_thread(get_sensors, ())
gps_thread = _thread.start_new_thread(get_gps, ())
link_thread = _thread.start_new_thread(data_link, ())

