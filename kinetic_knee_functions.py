'''
File contains main functions for the calculation of joint angles based on self-built inertial measurement units.

Functions are included for:
	* The calculation of the joint angle based on proximal and distal segment orientations.
	* Calculation the distal endpoints of a segment in the sagittal and frontal planes
	* Handling the initial relative orientation of one measurement unit to another
	* Finding and accounting for the initial orientation of the sensor relative to the plane of motion
'''


import serial
import time
import signal
import sys
import numpy as np
import matplotlib.pyplot as plt

def joint_angle(prox_X, prox_Y, prox_Z, dist_X, dist_Y, dist_Z):
	# Proximal segment
	alpha = np.radians(prox_X)
	beta = np.radians(prox_Y)
	gamma = np.radians(prox_Z)

	Rx = np.array([[1, 0, 0],[0, np.cos(alpha), np.sin(alpha)], [0, -np.sin(alpha), np.cos(alpha)]])
	Ry = np.array([[np.cos(beta), 0, -np.sin(beta)], [0, 1, 0], [np.sin(beta), 0, np.cos(beta)]])
	Rz = np.array([[np.cos(gamma), np.sin(gamma), 0], [-np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])

	R_PROX = np.dot(np.dot(Rz, Ry), Rx)

	# Distal segment

	alpha = np.radians(dist_X)
	beta = np.radians(dist_Y)
	gamma = np.radians(dist_Z)

	Rx = np.array([[1, 0, 0],[0, np.cos(alpha), np.sin(alpha)], [0, -np.sin(alpha), np.cos(alpha)]])
	Ry = np.array([[np.cos(beta), 0, -np.sin(beta)], [0, 1, 0], [np.sin(beta), 0, np.cos(beta)]])
	Rz = np.array([[np.cos(gamma), np.sin(gamma), 0], [-np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])

	R_DIST = np.dot(np.dot(Rz, Ry), Rx)

	# Joint Decomposition

	R_JOINT = np.dot(R_DIST, R_PROX.transpose())

	ALPHA_JOINT = np.degrees(np.arctan2(-R_JOINT[2,1],R_JOINT[2,2]))
	BETA_JOINT = np.degrees(np.arctan2(R_JOINT[2,0],np.sqrt((R_JOINT[0,0]**2 + R_JOINT[1,0]**2))))
	GAMMA_JOINT = np.degrees(np.arctan2(-R_JOINT[1,0],R_JOINT[0,0]))

	return ALPHA_JOINT, BETA_JOINT, GAMMA_JOINT

def sagittal_endpoints(X1, X2, C, H, L):
	prox_X = 90 - X1
	dist_X = 90 - X2

	prox_dY = np.round(np.sin(np.radians(prox_X)) * L)
	prox_dX = np.round(np.cos(np.radians(prox_X)) * L)

	dist_dY = np.round(np.sin(np.radians(dist_X)) * L)
	dist_dX = np.round(np.cos(np.radians(dist_X)) * L)

	dist_XE = C + dist_dX
	dist_YE = H - dist_dY

	prox_XE = dist_XE + prox_dX
	prox_YE = dist_YE - prox_dY

	return prox_XE, prox_YE, dist_XE, dist_YE

def frontal_endpoints(Y1, Y2, C, H, L, Side):
	# Note negative joint y = valgus
	# Intended to be mirror image of the motion
	if Side == 'R':
		prox_ang = 90 - Y1
		dist_ang = 90 + Y2
	elif Side == 'L':
		prox_ang = 90 + Y1
		dist_ang = 90 - Y2

	prox_dY = np.round(np.sin(np.radians(prox_ang)) * L)
	prox_dX = np.round(np.cos(np.radians(prox_ang)) * L)

	dist_dY = np.round(np.sin(np.radians(dist_ang)) * L)
	dist_dX = np.round(np.cos(np.radians(dist_ang)) * L)

	dist_XE = C + dist_dX
	dist_YE = H - dist_dY

	prox_XE = dist_XE + prox_dX
	prox_YE = dist_YE - prox_dY

	return prox_XE, prox_YE, dist_XE, dist_YE

def shift_Z(Z0, ZREF):
	# Switch both values to positive (Clockwise orientation)
	Z0 = np.absolute(Z0)
	ZREF = np.absolute(ZREF)

	if ZREF < 180:
		if Z0 >= ZREF and Z0 < ZREF + 180:
			Z = Z0 - ZREF
		elif Z0 < ZREF:
			Z = Z0 - ZREF
		elif Z0 >= (ZREF + 180):
			Z = (Z0 - 360) - ZREF
	elif ZREF >= 180:
		if (Z0 < ZREF) and Z0 >= (ZREF - 180):
			Z = Z0 - ZREF
		elif Z0 >= ZREF:
			Z = Z0 - ZREF
		elif Z0 < (ZREF - 180):
			Z = Z0 + (360 - ZREF)

	# Switch Z rotation back to right hand rule convention
	Z = -Z
	return Z

def orientation_matrix(alpha, beta, gamma):
	alpha = np.radians(alpha)
	beta = np.radians(beta)
	gamma = np.radians(gamma)

	Rx = np.array([[1, 0, 0],[0, np.cos(alpha), np.sin(alpha)], [0, -np.sin(alpha), np.cos(alpha)]])
	Ry = np.array([[np.cos(beta), 0, -np.sin(beta)], [0, 1, 0], [np.sin(beta), 0, np.cos(beta)]])
	Rz = np.array([[np.cos(gamma), np.sin(gamma), 0], [-np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])

	R = np.dot(np.dot(Rz, Ry), Rx)
	return R

def extract_angles(R):
	ALPHA = np.degrees(np.arctan2(-R[2,1],R[2,2]))
	BETA = np.degrees(np.arctan2(R[2,0],np.sqrt((R[0,0]**2 + R[1,0]**2))))
	GAMMA = np.degrees(np.arctan2(-R[1,0],R[0,0]))

	return ALPHA, BETA, GAMMA

def headingOptimization( Cal_X, Cal_Y, XG, YG, ZG):
	N = X1G.size
	XGR = np.zeros(N)
	YGR = np.zeros(N)
	ZGR = np.zeros(N)

	min_lateral_val = 999

	for k in range(1,45):
		alpha = Cal_X
		beta = Cal_Y
		gamma = k

		R_CAL = orientation_matrix(alpha, beta, gamma)

		for i in range(N):
			GYRO = np.array([[XG[i]],[YG[i]],[ZG[i]]])
			GYRO_R = np.dot(R_CAL.transpose(), GYRO)

			XGR[i] = GYRO_R[0,0]
			YGR[i] = GYRO_R[1,0]
			ZGR[i] = GYRO_R[2,0]

		X_MEAN = np.mean(np.absolute(XGR))
		Y_MEAN = np.mean(np.absolute(YGR))
		Z_MEAN = np.mean(np.absolute(ZGR))

		sum_lateral = Y_MEAN + Z_MEAN

		if sum_lateral < min_lateral_val:
			min_lateral_val = sum_lateral
			Z_OFFSET = k

	for k in range(1,45):
		k = -k
		alpha = Cal_X
		beta = Cal_Y
		gamma = k

		R_CAL = orientation_matrix(alpha, beta, gamma)

		for i in range(N):
			GYRO = np.array([[XG[i]],[YG[i]],[ZG[i]]])
			GYRO_R = np.dot(R_CAL.transpose(), GYRO)

			XGR[i] = GYRO_R[0,0]
			YGR[i] = GYRO_R[1,0]
			ZGR[i] = GYRO_R[2,0]

		X_MEAN = np.mean(np.absolute(XGR))
		Y_MEAN = np.mean(np.absolute(YGR))
		Z_MEAN = np.mean(np.absolute(ZGR))

		sum_lateral = Y_MEAN + Z_MEAN

		if sum_lateral < min_lateral_val:
			min_lateral_val = sum_lateral
			Z_OFFSET = k

	return Z_OFFSET		






