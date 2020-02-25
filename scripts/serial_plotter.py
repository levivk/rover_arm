#!/usr/bin/python

# This script is a modified version of one posted in a comment by McPeppr here: 
# https://create.arduino.cc/projecthub/highvoltages/arduino-real-time-plotting-with-python-5a6a5a 

import serial
import matplotlib.pyplot as plt
plt.ion() # ???? initial a figure
i=0
#ser = serial.Serial('COM13',9600) # Windows
#ser = serial.Serial('/dev/ttyAMA0',9600) # Raspi
ser = serial.Serial('/dev/ttyACM0',9600)
# ser = serial.Serial('/dev/ttyUSB0',9600) # Raspi
#ser = serial.Serial('/dev/ttyACM0',115200)
ser.close()
ser.open() # this will also reboot the arduino
data = float(ser.readline().decode().replace('\r', '').replace('\n', '')) # first data will not be plotted
try:
    while True:
        data = float(ser.readline().decode().replace('\r', '').replace('\n', ''))
        data2 = float(ser.readline().decode().replace('\r', '').replace('\n', ''))
        i += 1
        plt.title('serial reader: ' + str(data) + str(data2), loc='left')
        plt.plot(i, data, 'og') # pyplot will add this data
        plt.plot(i, data2, 'ob')
        plt.show() # update plot
        plt.pause(0.0001) # pause

except KeyboardInterrupt:
    ser.close()
    print("serial connection closed")
    #plt.close()
