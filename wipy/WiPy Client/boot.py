# boot.py -- run on boot-up

from network import WLAN

wlan = WLAN(mode=WLAN.STA, antenna=WLAN.INT_ANT)

wlan.connect('HawkWorks GS', auth=(WLAN.WPA2, 'aero2020'))

wlan.ifconfig(config=('192.168.4.2', '255.255.255.0', '192.168.4.1', '8.8.8.8'))

while not wlan.isconnected():
    pass

print(wlan.isconnected())