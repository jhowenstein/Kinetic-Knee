'''
Script collects data from knee sleeve with two inertial measurement units
	with no calibration protocol.
'''

import serial
import time
import signal
import sys
import numpy as np

def signal_handler(signal, frame):
    ser.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Serial Port for Bluetooth classic
#ser = serial.Serial('/dev/cu.AdafruitEZ-Linka7b1-SPP', 57600, timeout=1)

# Serial Port for feather board plugged in by USB (Right Mac USB Port)
#ser = serial.Serial('/dev/cu.usbmodem1421', 57600, timeout=1)

# Serial Port for feather board plugged in by USB (Left Mac USB Port)
ser = serial.Serial('/dev/cu.usbmodem1411', 57600, timeout=1)

time.sleep(.1)

# Serial Port for Start/Stop Button Remote Device
ser2 = serial.Serial('/dev/cu.usbserial-DN01DJY7', 57600, timeout=1)

print("Connection Successful")

filename = input("Filename?: ")

filename1 = filename + ".csv"

text_file1 = open(filename1, 'w')

while True:
	if ser2.inWaiting():
		line2 = ser2.readline().decode('utf-8')
		print(line2.strip())
		if line2.strip() == 'start':
			break

ser.write(b'0')

while True:
	if ser.inWaiting():
		line = ser.readline().decode('utf-8')
		print(line.strip())
		text_file1.write(line)
	if ser2.inWaiting():
		line2 = ser2.readline().decode('utf-8')
		print(line2.strip())
		if line2.strip() == 'stop':
			break

ser.write(b'1')

text_file1.close()

ser.close()
