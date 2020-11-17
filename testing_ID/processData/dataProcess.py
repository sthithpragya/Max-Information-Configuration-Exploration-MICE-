#!/usr/bin/env python
import numpy as np
import csv
import os.path
import pandas
import math
import numpy as np
import yaml
import io
import random
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# with open(r'/home/sthithpragya/Study/Thesis_test/catkin_ws/src/iiwa_ros/iiwa_data_collection/config/param.yaml') as stream:
with open(r'./../config/param.yaml') as stream:
	paramLoaded = yaml.safe_load(stream)

totalJoints = paramLoaded["totalJoints"]
savePath = paramLoaded["savePath"]
recordingResultantTorque = paramLoaded["recordingResultantTorque"]
learningMethod = paramLoaded["learningMethod"]

AJ = "recordedActualJointData"
AT = "computedActualTaskData"
DJ = "recordedDesiredJointData"
DT = "computedDesiredTaskData"

JT = "predJointTorque"
time = "recordedTimeData"

AJ = os.path.join(savePath, AJ + ".csv")
AT = os.path.join(savePath, AT + ".csv")
DJ = os.path.join(savePath, DJ + ".csv")
DT = os.path.join(savePath, DT + ".csv")
JT = os.path.join(savePath, JT + ".csv")

time = os.path.join(savePath, time + ".csv")

AJdata = pandas.read_csv(AJ, header=None, skiprows=1)
DJdata = pandas.read_csv(DJ, header=None, skiprows=1)
ATdata = pandas.read_csv(AT, header=None, skiprows=1)
DTdata = pandas.read_csv(DT, header=None, skiprows=1) 
# JTdata = pandas.read_csv(JT, header=None, skiprows=1) 
timeData = pandas.read_csv(time) # list of time floats

AJdata.index = range(len(AJdata))
DJdata.index = range(len(DJdata))
ATdata.index = range(len(ATdata))
DTdata.index = range(len(DTdata))
# JTdata.index = range(len(JTdata))
timeData.index = range(len(timeData))

toFlip = False
if learningMethod == "URDF":
	toFlip = False
else:
	if recordingResultantTorque:
		toFlip = True
	else:
		toFlip = False


#--------------------------------------------------------------

infoFile = os.path.join(savePath, "info" + ".txt") 
info = open(infoFile, "w")

qMSE = [0 for i in range(totalJoints)]
qDotMSE = [0 for i in range(totalJoints)]

qMAE = [0 for i in range(totalJoints)]
qDotMAE = [0 for i in range(totalJoints)]

tauMSE = [0 for i in range(totalJoints)]
tauMAE = [0 for i in range(totalJoints)]

tauMean = [0 for i in range(totalJoints)]
tauStd = [0 for i in range(totalJoints)]

for jointIndex in range(totalJoints):

	qMSE[jointIndex] = np.sqrt((((AJdata.iloc[:,jointIndex] - DJdata.iloc[:,jointIndex]).pow(2)).sum(axis=0))/len(AJdata))
	qDotMSE[jointIndex] = np.sqrt((((AJdata.iloc[:,totalJoints+jointIndex] - DJdata.iloc[:,totalJoints+jointIndex]).pow(2)).sum(axis=0))/len(AJdata))

	qMAE[jointIndex] = (((AJdata.iloc[:,jointIndex] - DJdata.iloc[:,jointIndex]).abs()).sum(axis=0))/len(AJdata)
	qDotMAE[jointIndex] = (((AJdata.iloc[:,totalJoints+jointIndex] - DJdata.iloc[:,totalJoints+jointIndex]).abs()).sum(axis=0))/len(AJdata)

	# if toFlip:
		# Since the predictions are opposite in sign to sensor readings
		# tauMAE[jointIndex] = (((AJdata.iloc[:,2*totalJoints+jointIndex] + JTdata.iloc[:,jointIndex]).abs()).sum(axis=0))/len(AJdata)
		# tauMSE[jointIndex] = np.sqrt((((AJdata.iloc[:,2*totalJoints+jointIndex] + JTdata.iloc[:,jointIndex]).pow(2)).sum(axis=0))/len(AJdata))
		# tauMean[jointIndex] = (((AJdata.iloc[:,2*totalJoints+jointIndex] + JTdata.iloc[:,jointIndex])).mean(axis=0))
		# tauStd[jointIndex] = (((AJdata.iloc[:,2*totalJoints+jointIndex] + JTdata.iloc[:,jointIndex])).std(axis=0))

	# else:
		# tauMAE[jointIndex] = (((AJdata.iloc[:,2*totalJoints+jointIndex] - JTdata.iloc[:,jointIndex]).abs()).sum(axis=0))/len(AJdata)
		# tauMSE[jointIndex] = np.sqrt((((AJdata.iloc[:,2*totalJoints+jointIndex] - JTdata.iloc[:,jointIndex]).pow(2)).sum(axis=0))/len(AJdata))
		# tauMean[jointIndex] = (((AJdata.iloc[:,2*totalJoints+jointIndex] - JTdata.iloc[:,jointIndex])).mean(axis=0))
		# tauStd[jointIndex] = (((AJdata.iloc[:,2*totalJoints+jointIndex] - JTdata.iloc[:,jointIndex])).std(axis=0))
	   

fbTauMag = [0 for i in range(totalJoints)]

# for i in range(totalJoints):
	# fbTauMag[i] = (JTdata.iloc[:,totalJoints+i].abs().sum(axis=0))/len(AJdata)

info.write("Joint angle root mean sq. error:    " + str(qMSE) + "\n")
info.write("Joint velocity root mean sq. error: " + str(qDotMSE) + "\n")
info.write("   ")
info.write("Joint angle root mean abs error:    " + str(qMAE) + "\n")
info.write("Joint velocity mean abs error: " + str(qDotMAE) + "\n")
info.write("   ")
info.write("Joint torque root mean sq. error:    " + str(tauMSE) + "\n")
info.write("Joint torque mean abs error: " + str(tauMAE) + "\n")
info.write("   ")
info.write("Joint torque mean error:    " + str(tauMean) + "\n")
info.write("Joint torque std error: " + str(tauStd) + "\n")
info.write("   ")
info.write("Mean absolute FB torques: " +  str(fbTauMag) + "\n")
info.close()

print("Joint angle root mean sq. error:    " + str(qMSE) + "\n")
print("Joint velocity root mean sq. error: " + str(qDotMSE) + "\n")
######## Visualisation

# Plot q vs t
for jointIndex in range(totalJoints):
	plt.figure(10+jointIndex)
	plt.scatter(timeData.iloc[:,0], AJdata.iloc[:,jointIndex], label='Actual')
	plt.scatter(timeData.iloc[:,0], DJdata.iloc[:,jointIndex], label='Desired')
	plt.xlabel('time')
	plt.ylabel('joint angle')
	plt.legend()
	fileName = os.path.join(savePath, str(20+jointIndex) + '.png')
	# plt.savefig(fileName, bbox_inches='tight')

# Plot qDot vs t
for jointIndex in range(totalJoints):
	plt.figure(20+jointIndex)
	plt.plot(timeData.iloc[:,0].tolist(), AJdata.iloc[:,totalJoints+jointIndex].tolist(), label='qDot actual')
	plt.plot(timeData.iloc[:,0].tolist(), DJdata.iloc[:,totalJoints+jointIndex].tolist(), label='qDot desired')
	plt.xlabel('time')
	plt.ylabel('joint velocity')
	plt.legend()
	fileName = os.path.join(savePath, str(50+jointIndex) + '.png')
	# plt.savefig(fileName, bbox_inches='tight')


# for jointIndex in range(totalJoints):
# 	plt.figure(30+jointIndex)
# 	if toFlip:
# 		tauError = AJdata.iloc[:,2*totalJoints+jointIndex] + JTdata.iloc[:,jointIndex]
# 	else:
# 		tauError = AJdata.iloc[:,2*totalJoints+jointIndex] - JTdata.iloc[:,jointIndex]
# 	tauError = tauError.abs

# 	if toFlip:
# 		plt.plot((-JTdata.iloc[:,jointIndex]).tolist(), label='pred torque')
# 	else:
# 		plt.plot((JTdata.iloc[:,jointIndex]).tolist(), label='pred torque')
# 	plt.plot(AJdata.iloc[:,2*totalJoints+jointIndex].tolist(), label='actual torque')
# 	plt.xlabel('time')
# 	plt.ylabel('mean absolute torque error')
# 	plt.legend()
# 	fileName = os.path.join(savePath, str(70+jointIndex) + '.png')
# 	# plt.savefig(fileName, bbox_inches='tight')

for jointIndex in range(totalJoints):
	plt.figure(30+jointIndex)
	plt.plot(AJdata.iloc[:,2*totalJoints+jointIndex].tolist(), label='actual torque')
	plt.xlabel('time')
	plt.ylabel('mean absolute torque error')
	plt.legend()
	fileName = os.path.join(savePath, str(70+jointIndex) + '.png')
	# plt.savefig(fileName, bbox_inches='tight')

plt.show()