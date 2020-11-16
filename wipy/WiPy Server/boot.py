import machine
import network as nw
import time

machine.freq(240000000)

wlan = nw.WLAN(nw.AP_IF)
wlan.active(True)
nw.phy_mode(nw.AP_IF, nw.MODE_LR)

wlan.config(essid='HawkWorks GS', channel=11, password='aero2020', authmode=3)

time.sleep(1)