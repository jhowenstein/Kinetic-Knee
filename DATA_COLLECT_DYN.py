'''
Script collects data from knee sleeve with two inertial measurement units
	with a dynamic and static calibration routine.
'''

import serial
import time
import signal
import sys
import numpy as np
from kinetic_knee_functions import *

LINE_LENGTH = 13

def signal_handler(signal, frame):
    ser.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Serial Port for Bluetooth classic
ser = serial.Serial('/dev/cu.AdafruitEZ-Linka7b1-SPP', 57600, timeout=1)

# Serial Port for feather board plugged in by USB (Right Mac USB Port)
#ser = serial.Serial('/dev/cu.usbmodem1421', 57600, timeout=1)

# Serial Port for feather board plugged in by USB (Left Mac USB Port)
#ser = serial.Serial('/dev/cu.usbmodem1411', 57600, timeout=1)

time.sleep(.1)

# Serial Port for Start/Stop Button Remote Device
ser2 = serial.Serial('/dev/cu.usbserial-DN01DJY7', 57600, timeout=1)

print("Connection Successful")

filename = input("Filename?: ")

data_filename = filename + ".csv"
cal_filename = filename + "_cal.csv"
dynamic_filename = filename + "_dynamic.csv"
coeff_filename = filename + "_coeff.csv"

while True:
	if ser2.inWaiting():
		line2 = ser2.readline().decode('utf-8')
		print(line2.strip())
		if line2.strip() == 'start':
			break

print("Static calibration begin in: ")
countdown = 3
while countdown > 0:
	print(countdown)
	countdown += -1
	time.sleep(1)

nCal = 0
endCal = 50
first_line = 1

ser.write(b'1')

while nCal < endCal:
	if ser.inWaiting():
		line = ser.readline().decode('utf-8')
		S = line.strip()
		print(S)
		S = S.split(',')
		S = list(map(float, S))
		cal_row = np.array(S)
		if cal_row.size == LINE_LENGTH:
			if first_line == 1:
				static = cal_row
				first_line += 1
			else:
				static = np.vstack((static,cal_row))
			nCal += 1


ser.write(b'0')

cal_coeff = np.mean(static, axis=0)

print("Static calibration complete.")
print("Dynamic calibration begin in: ")
countdown = 3
while countdown > 0:
	print(countdown)
	countdown += -1
	time.sleep(1)

nCal = 0
endCal = 200
first_line = 1

ser.write(b'1')

while nCal < endCal:
	if ser.inWaiting():
		line = ser.readline().decode('utf-8')
		S = line.strip()
		print(S)
		S = S.split(',')
		S = list(map(float, S))
		cal_row = np.array(S)
		if cal_row.size == LINE_LENGTH:
			if first_line == 1:
				dynamic = cal_row
				first_line += 1
			else:
				dynamic = np.vstack((dynamic,cal_row))
			nCal += 1

ser.write(b'0')

print("Calibration complete.")
print("Data Collection begin in: ")
countdown = 3
while countdown > 0:
	print(countdown)
	countdown += -1
	time.sleep(1)

ser.write(b'1')

first_line = 0
count = -1
while True:
	if ser.inWaiting():
		if first_line == 0:
			line = ser.readline().decode('utf-8')
			S = line.strip()
			print(S)
			S = S.split(',')
			S = list(map(float, S))
			output_row = np.array(S)
			if output_row[0] == 1:
				output = output_row
				first_line += 1
		else:
			line = ser.readline().decode('utf-8')
			S = line.strip()
			print(S)
			S = S.split(',')
			S = list(map(float, S))
			output_row = np.array(S)
			if output_row.size == LINE_LENGTH:
				output = np.vstack((output,output_row))

	if ser2.inWaiting():
		line2 = ser2.readline().decode('utf-8')
		if line2.strip() == 'stop':
			break

ser.write(b'0')

ser.close()

np.savetxt(data_filename, output, delimiter=',')
np.savetxt(cal_filename, static, delimiter=',')
np.savetxt(dynamic_filename, dynamic, delimiter=',')
np.savetxt(coeff_filename, cal_coeff, delimiter=',')