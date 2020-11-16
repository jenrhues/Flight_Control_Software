# main.py -- put your code here!

import socket
import _thread
import time
from machine import UART

uart = UART(1, 400000)
uart.init(bits=8, parity=None, stop=1, tx=25, rx=26)

ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)

ss.bind(('', 5000))

ss.listen(5)

send_lock = _thread.allocate_lock()

gliders = dict()

def serv_sock(sock, lst):

    while True:
        cs = sock.accept()
        cs[0].settimeout(5)

        lst[cs[1]] = cs[0]

        _thread.start_new_thread(cli_sock, (cs,))


    

def cli_sock(sock):

    #pycom.rgbled(0x00FF00)
    #print("New client connected at address: " + str(sock[1]))

    while True:
        data = sock[0].readline()
        if data != None:
            #print(str(sock[1]) + ": " + data.decode('utf-8'))
            
            if send_lock.acquire(0):
                #print(uart.read())
                #uart.write('stuff and things')
                #msg = str(sock[1][0]) + ":" + data.decode('utf-8')
                uart.write(data)
                send_lock.release()

t = _thread.start_new_thread(serv_sock, (ss, gliders))

