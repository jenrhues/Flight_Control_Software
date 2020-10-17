from network import WLAN
import pycom
import time

pycom.heartbeat(False)

wlan = WLAN(mode=WLAN.AP, ssid='HawkWorks GS', auth=(WLAN.WPA2, 'aero2020'),  antenna=WLAN.INT_ANT)
wlan.ifconfig(id=1)

time.sleep(5)

pycom.rgbled(0xFF00FF)