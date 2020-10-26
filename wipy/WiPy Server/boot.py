from network import WLAN
import pycom
import time

pycom.heartbeat(False)

wlan = WLAN(mode=WLAN.AP, ssid='HawkWorks GS', auth=(WLAN.WPA2, 'aero2020'),  antenna=WLAN.EXT_ANT)
wlan.ifconfig(id=1)
wlan.wifi_protocol((0,0,0,1))

time.sleep(1)

pycom.rgbled(0xFF00FF)