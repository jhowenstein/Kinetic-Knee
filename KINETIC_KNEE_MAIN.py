'''
This is the main collection and analysis file to track knee kinematic data using a knee sleeve with two embedded IMU's.

This script recieves data from the two inertial measurement sensors and displays the knee orientation
from the sagittal and frontal planes. The script stores the data as it is being displayed then runs a
further calculations and analysis after the movement trial has been completed. The results are displayed
in graph form at the end.
'''

import serial
import time
import signal
import sys
import numpy as np
import pygame, sys
from pygame.locals import *
from kinetic_knee_functions import *

LINE_LENGTH = 13
DYNAMIC_CALIBRATION = True
PLOTS_ON = True

dt = .02
TIME_SCALE = 2

def signal_handler(signal, frame):
    ser.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

WINDOW_HEIGHT = 801
WINDOW_WIDTH = 1201
C1 = 300
C2 = 900
H = int((WINDOW_HEIGHT-1) * .9)
L = int(np.round(C1 * .75))

# Serial Port for Bluetooth classic
ser = serial.Serial('/dev/cu.AdafruitEZ-Linka7b1-SPP', 57600, timeout=1)

# Serial Port for feather board plugged in by USB
#ser = serial.Serial('/dev/cu.usbmodem1411', 57600, timeout=1)

time.sleep(.1)

# Serial Port for Start/Stop Button Remote Device
ser2 = serial.Serial('/dev/cu.usbserial-DN01DJY7', 57600, timeout=1)

print("Bluetooth Connection Successful")

filename = input("Filename?: ")

data_file = filename + ".csv"
cal_file = filename + "_cal.csv"
dynamic_file = filename + "_dynamic.csv"
coeff_file = filename + "_coeff.csv"

pygame.init()

DISPLAYSURF = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption('Kinetic Knee')
DISPLAYSURF.fill(BLACK)
pygame.display.update()

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

ser.write(b'0')

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

ser.write(b'1')

cal_coeff = np.mean(static, axis=0)

print("Calibration complete.")

Cal_X1 = cal_coeff[3]
Cal_Y1 = cal_coeff[2]
Cal_Z1 = cal_coeff[1]
Cal_X2 = cal_coeff[9]
Cal_Y2 = cal_coeff[8]
Cal_Z2 = cal_coeff[7]

alpha = Cal_X1
beta = Cal_Y1
gamma = 0
R_CAL1 = orientation_matrix(alpha, beta, gamma)

alpha = Cal_X2
beta = Cal_Y2
gamma = 0
R_CAL2 = orientation_matrix(alpha, beta, gamma)

if DYNAMIC_CALIBRATION:
	print("Dynamic calibration begin in: ")
	countdown = 3
	while countdown > 0:
		print(countdown)
		countdown += -1
		time.sleep(1)

	nCal = 0
	endCal = 200
	first_line = 1

	ser.write(b'0')

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

	ser.write(b'1')
	print("Calibration complete.")


if DYNAMIC_CALIBRATION:
	X1G_DYN = dynamic[:,4]
	Y1G_DYN = dynamic[:,5]
	Z1G_DYN = dynamic[:,6]

	X2G_DYN = dynamic[:,10]
	Y2G_DYN = dynamic[:,11]
	Z2G_DYN = dynamic[:,12]

	Z1_OFFSET = headingOptimization(Cal_X1, Cal_Y1, X1G_DYN, Y1G_DYN, Z1G_DYN)
	Z2_OFFSET = headingOptimization(Cal_X2, Cal_Y2, X2G_DYN, Y2G_DYN, Z2G_DYN)


print("Data Collection begin in: ")
countdown = 3
while countdown > 0:
	print(countdown)
	countdown += -1
	time.sleep(1)

ser.write(b'0')

X1ROT = 0
Y1ROT = 0
Z1ROT = 0
X2ROT = 0
Y2ROT = 0
Z2ROT = 0

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
				Z1REF = output_row[1]
				Z2REF = output_row[7]
		else:
			line = ser.readline().decode('utf-8')
			S = line.strip()
			print(S)
			S = S.split(',')
			S = list(map(float, S))
			output_row = np.array(S)
			if output_row.size == LINE_LENGTH:
				output = np.vstack((output,output_row))
				count += 1
				if count%2 == 0:
					time = output_row[0]
					X1 = output_row[3]
					Y1 = output_row[2]
					Z1 = output_row[1]
					X1G = output_row[4]
					Y1G = output_row[5]
					Z1G = output_row[6]
					X2 = output_row[9]
					Y2 = output_row[8]
					Z2 = output_row[7]
					X2G = output_row[10]
					Y2G = output_row[11]
					Z2G = output_row[12]

					Z1 = shift_Z(Z1, Z1REF)
					Z2 = shift_Z(Z2, Z2REF)
					# Proximal Segment
					alpha = Cal_X1
					beta = Cal_Y1
					gamma = 0

					R_STATIC1 = orientation_matrix(alpha, beta, gamma)

					# Distal Segment
					alpha = Cal_X2
					beta = Cal_Y2
					gamma = 0

					R_STATIC2 = orientation_matrix(alpha, beta, gamma)

					alpha = X1
					beta = Y1
					gamma = Z1

					R_RAW1 = orientation_matrix(alpha, beta, gamma)

					R = np.dot(R_RAW1, R_STATIC1.transpose())

					X1R, Y1R, Z1R = extract_angles(R)

					alpha = X2
					beta = Y2
					gamma = Z2

					R_RAW2 = orientation_matrix(alpha, beta, gamma)

					R = np.dot(R_RAW2, R_STATIC2.transpose())

					X2R, Y2R, Z2R = extract_angles(R)

					alpha = Cal_X1
					beta = Cal_Y1
					gamma = 0
					if DYNAMIC_CALIBRATION:
						gamma = Z1_OFFSET

					R_CAL1 = orientation_matrix(alpha, beta, gamma)

					alpha = Cal_X2
					beta = Cal_Y2
					gamma = 0
					if DYNAMIC_CALIBRATION:
						gamma = Z2_OFFSET

					R_CAL2 = orientation_matrix(alpha, beta, gamma)

					alpha = X1ROT
					beta = Y1ROT
					gamma = Z1ROT

					R_CURR = orientation_matrix(alpha, beta, gamma)

					GYRO = np.array([[X1G],[Y1G],[Z1G]])
					GYRO_R = np.dot(R_CAL1.transpose(), GYRO)
					GYRO_R = np.dot(R_CURR.transpose(), GYRO_R)

					X1GR = GYRO_R[0,0]
					Y1GR = GYRO_R[1,0]
					Z1GR = GYRO_R[2,0]

					X1ROT = X1ROT + ( X1GR * dt * 57.3 * TIME_SCALE)
					Y1ROT = Y1ROT + ( Y1GR * dt * 57.3 * TIME_SCALE)
					Z1ROT = Z1ROT + ( Z1GR * dt * 57.3 * TIME_SCALE)

					alpha = X2ROT
					beta = Y2ROT
					gamma = Z2ROT

					R_CURR = orientation_matrix(alpha, beta, gamma)

					GYRO = np.array([[X2G],[Y2G],[Z2G]])
					GYRO_R = np.dot(R_CAL1.transpose(), GYRO)
					GYRO_R = np.dot(R_CURR.transpose(), GYRO_R)

					X2GR = GYRO_R[0,0]
					Y2GR = GYRO_R[1,0]
					Z2GR = GYRO_R[2,0]

					X2ROT = X2ROT + ( X2GR * dt * 57.3 * TIME_SCALE)
					Y2ROT = Y2ROT + ( Y2GR * dt * 57.3 * TIME_SCALE)
					Z2ROT = Z2ROT + ( Z2GR * dt * 57.3 * TIME_SCALE)

					hip_x, hip_y, hip_z = joint_angle(0, 0, 0, X1ROT, Y1ROT, Z1ROT)
					knee_x, knee_y, knee_z = joint_angle(X1ROT, Y1ROT, Z1ROT, X2ROT, Y2ROT, Z2ROT)

					prox_XE, prox_YE, dist_XE, dist_YE = sagittal_endpoints(X1ROT, X2ROT, C1, H, L)
					front_prox_XE, front_prox_YE, front_dist_XE, front_dist_YE = frontal_endpoints(Y1ROT, Y2ROT, C2, H, L, 'R')

					DISPLAYSURF.fill(BLACK)
					pygame.draw.line(DISPLAYSURF, WHITE, (C1, H), (dist_XE, dist_YE), 3)
					pygame.draw.line(DISPLAYSURF, WHITE, (dist_XE, dist_YE), (prox_XE, prox_YE), 3)

					pygame.draw.line(DISPLAYSURF, WHITE, (C2, H), (front_dist_XE, front_dist_YE), 3)
					pygame.draw.line(DISPLAYSURF, WHITE, (front_dist_XE, front_dist_YE), (front_prox_XE, front_prox_YE), 3)
					pygame.display.update()

	if ser2.inWaiting():
		line2 = ser2.readline().decode('utf-8')
		if line2.strip() == 'stop':
			break

	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()


ser.write(b'1')

pygame.quit()

ser.close()

np.savetxt(data_file, output, delimiter=',')
np.savetxt(cal_file, static, delimiter=',')
np.savetxt(coeff_file, cal_coeff, delimiter=',')
np.savetxt(dynamic_file, dynamic, delimiter=',')

Cal_X1 = cal_coeff[3]
Cal_Y1 = cal_coeff[2]
Cal_Z1 = cal_coeff[1]

Cal_X2 = cal_coeff[9]
Cal_Y2 = cal_coeff[8]
Cal_Z2 = cal_coeff[7]

time = output[:,0]
N = time.size

X1 = output[:,3]
Y1 = output[:,2]
Z1 = output[:,1]

X1G = output[:,4]
Y1G = output[:,5]
Z1G = output[:,6]

X2 = output[:,9]
Y2 = output[:,8]
Z2 = output[:,7]

X2G = output[:,11]
Y2G = output[:,12]
Z2G = output[:,13]

X1ROT = np.zeros(N)
Y1ROT = np.zeros(N)
Z1ROT = np.zeros(N)

X2ROT = np.zeros(N)
Y2ROT = np.zeros(N)
Z2ROT = np.zeros(N)

Z1REF = Z1[0] #Replaced with reference point upon trigger in live motion trial
Z2REF = Z2[0]

for i in range(N):
	Z1[i] = shift_Z(Z1[i],Z1REF)
	Z2[i] = shift_Z(Z2[i],Z2REF)

alpha = Cal_X1
beta = Cal_Y1
gamma = 0

R_CAL1 = orientation_matrix(alpha, beta, gamma)

alpha = Cal_X2
beta = Cal_Y2
gamma = 0

R_CAL2 = orientation_matrix(alpha, beta, gamma)

X1R = np.zeros(N)
Y1R = np.zeros(N)
Z1R = np.zeros(N)

X2R = np.zeros(N)
Y2R = np.zeros(N)
Z2R = np.zeros(N)

for i in range(N):
	alpha = X1[i]
	beta = Y1[i]
	gamma = Z1[i]

	R_RAW = orientation_matrix(alpha, beta, gamma)

	R = np.dot(R_RAW, R_CAL1.transpose())

	X1R[i], Y1R[i], Z1R[i] = extract_angles(R)

	alpha = X2[i]
	beta = Y2[i]
	gamma = Z2[i]

	R_RAW = orientation_matrix(alpha, beta, gamma)

	R = np.dot(R_RAW, R_CAL2.transpose())

	X2R[i], Y2R[i], Z2R[i] = extract_angles(R)


if DYNAMIC_CALIBRATION:
	X1G_DYN = dynamic[:,4]
	Y1G_DYN = dynamic[:,5]
	Z1G_DYN = dynamic[:,6]

	X2G_DYN = dynamic[:,10]
	Y2G_DYN = dynamic[:,11]
	Z2G_DYN = dynamic[:,12]

	Z1_OFFSET = headingOptimization(Cal_X1, Cal_Y1, X1G_DYN, Y1G_DYN, Z1G_DYN)
	Z2_OFFSET = headingOptimization(Cal_X2, Cal_Y2, X2G_DYN, Y2G_DYN, Z2G_DYN)

alpha = Cal_X1
beta = Cal_Y1
gamma = 0
if DYNAMIC_CALIBRATION:
	gamma = Z1_OFFSET

R_CAL1 = orientation_matrix(alpha, beta, gamma)

alpha = Cal_X2
beta = Cal_Y2
gamma = 0
if DYNAMIC_CALIBRATION:
	gamma = Z2_OFFSET

R_CAL2 = orientation_matrix(alpha, beta, gamma)

X1GR = np.zeros(N)
Y1GR = np.zeros(N)
Z1GR = np.zeros(N)

X2GR = np.zeros(N)
Y2GR = np.zeros(N)
Z2GR = np.zeros(N)

for i in range(1,N):
	alpha = X1ROT[i-1]
	beta = Y1ROT[i-1]
	gamma = Z1ROT[i-1]

	R_CURR = orientation_matrix(alpha, beta, gamma)

	GYRO = np.array([[X1G[i]],[Y1G[i]],[Z1G[i]]])
	GYRO_R = np.dot(R_CAL1.transpose(), GYRO)
	GYRO_R = np.dot(R_CURR.transpose(), GYRO_R)

	X1GR[i] = GYRO_R[0,0]
	Y1GR[i] = GYRO_R[1,0]
	Z1GR[i] = GYRO_R[2,0]

	X1ROT[i] = X1ROT[i-1] + ( X1GR[i] * dt * 57.3 )
	Y1ROT[i] = Y1ROT[i-1] + ( Y1GR[i] * dt * 57.3 )
	Z1ROT[i] = Z1ROT[i-1] + ( Z1GR[i] * dt * 57.3 )

	alpha = X2ROT[i-1]
	beta = Y2ROT[i-1]
	gamma = Z2ROT[i-1]

	R_CURR = orientation_matrix(alpha, beta, gamma)

	GYRO = np.array([[X2G[i]],[Y2G[i]],[Z2G[i]]])
	GYRO_R = np.dot(R_CAL2.transpose(), GYRO)
	GYRO_R = np.dot(R_CURR.transpose(), GYRO_R)

	X2GR[i] = GYRO_R[0,0]
	Y2GR[i] = GYRO_R[1,0]
	Z2GR[i] = GYRO_R[2,0]

	X2ROT[i] = X2ROT[i-1] + ( X2GR[i] * dt * 57.3 )
	Y2ROT[i] = Y2ROT[i-1] + ( Y2GR[i] * dt * 57.3 )
	Z2ROT[i] = Z2ROT[i-1] + ( Z2GR[i] * dt * 57.3 )

hip_x = np.zeros(N)
hip_y = np.zeros(N)
hip_z = np.zeros(N)
knee_x = np.zeros(N)
knee_y = np.zeros(N)
knee_z = np.zeros(N)

for i in range(N):
	hip_x, hip_y, hip_z = joint_angle(0, 0, 0, X1ROT[i], Y1ROT[i], Z1ROT[i])
	knee_x, knee_y, knee_z = joint_angle(X1ROT[i], Y1ROT[i], Z1ROT[i], X2ROT[i], Y2ROT[i], Z2ROT[i])


if PLOTS_ON:
	plt.figure(1)
	plt.plot(time,X1,time,Y1,time,Z1)
	plt.title("Sensor 1: Orientations")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(2)
	plt.plot(time,X2,time,Y2,time,Z2)
	plt.title("Sensor 2: Orientations")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(3)
	plt.plot(time,X1G,time,Y1G,time,Z1G)
	plt.title("Sensor 1: Original Gyro")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(4)
	plt.plot(time,X2G,time,Y2G,time,Z2G)
	plt.title("Sensor 2: Original Gyro")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(5)
	plt.plot(time,X1ROT,time,Y1ROT,time,Z1ROT)
	plt.title("Sensor 1: Gyro Derived Orientations")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(6)
	plt.plot(time,X2ROT,time,Y2ROT,time,Z2ROT)
	plt.title("Sensor 2: Gyro Derived Orientations")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(7)
	plt.plot(time,X1GR,time,Y1GR,time,Z1GR)
	plt.title("Sensor 1: Rotated Gyro")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(8)
	plt.plot(time,X2GR,time,Y2GR,time,Z2GR)
	plt.title("Sensor 2: Rotated Gyro")
	plt.legend(['X','Y','Z'])

if PLOTS_ON:
	plt.figure(9)
	plt.subplot(311)
	plt.plot(time,X1R,time,X1ROT)
	plt.title('X Orientation')
	plt.legend(['Original','Rotated'])

	plt.subplot(312)
	plt.plot(time,Y1R,time,Y1ROT)
	plt.title('Y Orientation')
	plt.legend(['Original','Rotated'])

	plt.subplot(313)
	plt.plot(time,Z1R,time,Z1ROT)
	plt.title('Z Orientation')
	plt.legend(['Original','Rotated'])
	plt.show()

if PLOTS_ON:
	plt.figure(10)
	plt.subplot(311)
	plt.plot(time,X2R,time,X2ROT)
	plt.title('X Orientation')
	plt.legend(['Original','Rotated'])

	plt.subplot(312)
	plt.plot(time,Y2R,time,Y2ROT)
	plt.title('Y Orientation')
	plt.legend(['Original','Rotated'])

	plt.subplot(313)
	plt.plot(time,Z2R,time,Z2ROT)
	plt.title('Z Orientation')
	plt.legend(['Original','Rotated'])
	plt.show()

if PLOTS_ON:
	plt.figure(11)
	plt.subplot(311)
	plt.plot(time,hip_x)
	plt.title('Joint X Rotation')

	plt.subplot(312)
	plt.plot(time,hip_y)
	plt.title('Joint Y Rotation')

	plt.subplot(313)
	plt.plot(time,hip_z)
	plt.title('Joint Z Rotation')
	plt.show()

if PLOTS_ON:
	plt.figure(12)
	plt.subplot(311)
	plt.plot(time,knee_x)
	plt.title('Joint X Rotation')

	plt.subplot(312)
	plt.plot(time,knee_y)
	plt.title('Joint Y Rotation')

	plt.subplot(313)
	plt.plot(time,knee_z)
	plt.title('Joint Z Rotation')
	plt.show()
