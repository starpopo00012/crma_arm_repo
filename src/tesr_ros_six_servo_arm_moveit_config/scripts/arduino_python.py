#create python code that send string to arduino

import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

while True:
        #input string
        angle = input("Enter angle: ")
        ser.write(angle.encode())
        print(angle)

        break