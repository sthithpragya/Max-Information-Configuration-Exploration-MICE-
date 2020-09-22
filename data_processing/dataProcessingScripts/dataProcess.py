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


with open(r'param.yaml') as stream:
	paramLoaded = yaml.safe_load(stream)


savePath = paramLoaded["savePath"]
useArtificialVel = paramLoaded["useArtVel"]
learnGravComp = paramLoaded["learnGravComp"]
totalJoints = paramLoaded["totalJoints"]

qLimLower = paramLoaded["qLimLower"]
qLimUpper = paramLoaded["qLimUpper"]

qDotLimLower = paramLoaded["qDotLimLower"]
qDotLimUpper = paramLoaded["qDotLimUpper"]

qBound = paramLoaded["qBound"]
qDotBound = paramLoaded["qDotBound"]
qMargin = paramLoaded["qMargin"]
qDotMargin = paramLoaded["qDotMargin"]

AJ = "recordedActualJointData"
AT = "computedActualTaskData"
DJ = "recordedDesiredJointData"
DT = "computedDesiredTaskData"
time = "recordedTimeData"
AJacc = "computedActualJointAccelarationData"
PAD = "processedActualData"
PDD = "processedDesiredData"

AJ = os.path.join(savePath, AJ + ".csv")
AJacc = os.path.join(savePath, AJacc + ".csv")
AT = os.path.join(savePath, AT + ".csv")
DJ = os.path.join(savePath, DJ + ".csv")
DT = os.path.join(savePath, DT + ".csv")
time = os.path.join(savePath, time + ".csv")
PAD = os.path.join(savePath, PAD + ".csv")
PDD = os.path.join(savePath, PDD + ".csv")

jointAngleNames = [[] for i in range(totalJoints)]
jointVelNames = [[] for i in range(totalJoints)]
jointTorqueNames = [[] for i in range(totalJoints)]
jointAccNames = [[] for i in range(totalJoints)]
taskColNames = ['X', 'Y', 'Z']
timeName = ['time']

for jointIndex in range(totalJoints):
	jointAngleNames[jointIndex] = 'J' + str(jointIndex+1)
	jointVelNames[jointIndex] = 'J'+ str(jointIndex+1) + 'dot'
	jointAccNames[jointIndex] = 'J'+ str(jointIndex+1) + 'ddot'
	jointTorqueNames[jointIndex] = 'Tau'+ str(jointIndex+1)
	
#Opening the recorded datas
AJdata = pandas.read_csv(AJ, names=jointAngleNames + jointVelNames + jointTorqueNames)
DJdata = pandas.read_csv(DJ, names=jointAngleNames + jointVelNames)
ATdata = pandas.read_csv(AT, names=taskColNames)
DTdata = pandas.read_csv(DT, names=taskColNames)
timeData = pandas.read_csv(time, names=timeName)  # list of time floats

#Removing the first row since it contains 0 or garbage entries
AJdata = AJdata.drop(0, axis=0)
DJdata = DJdata.drop(0, axis=0)
ATdata = ATdata.drop(0, axis=0)
DTdata = DTdata.drop(0, axis=0)
timeData = timeData.drop(0, axis=0)

#Re-index the data
AJdata.index = range(len(AJdata))
DJdata.index = range(len(DJdata))
ATdata.index = range(len(ATdata))
DTdata.index = range(len(DTdata))
timeData.index = range(len(timeData))

def divByTime(dataList):
	return (np.true_divide(np.diff(np.asarray([dataList[0]]+dataList)), np.diff([timeData.time.tolist()[0]]+timeData.time.tolist()))).reshape((len(dataList), 1))

# If the recorded data lacks joint-velocities OR the joint-velocities are unreliable, we compute the velocities from joint angle and time elapsed data (dQ/dT)
if useArtificialVel:
	ArtificialVel = "computedActualVelocityData"
	ArtificialVel = os.path.join(savePath, ArtificialVel + ".csv")

	with open(ArtificialVel, "w") as velCsv:
		pass

	Jdot = np.concatenate((divByTime(AJdata.iloc[:,0].tolist()), divByTime(AJdata.iloc[:,1].tolist())), axis=1)

	if(totalJoints > 2):
		jointIndex = 2
		while(jointIndex < totalJoints):
			Jdot = np.concatenate((Jdot, divByTime(AJdata.iloc[:,jointIndex].tolist())), axis=1)
		 
	Jdot[Jdot == -np.inf] = 0  # replacing inf with 0 for later removal
	Jdot[Jdot == np.inf] = 0  # replacing inf with 0 for later removal

	np.savetxt(ArtificialVel, Jdot, delimiter=",")

	ArtificialVelData = pandas.read_csv(ArtificialVel, names=jointVelNames)

	nanRowsVel = ArtificialVelData.T.isnull().any()

	ArtificialVelData = ArtificialVelData[~nanRowsVel]
	AJdata = AJdata[~nanRowsVel]
	timeData = timeData[~nanRowsVel]
	DJdata = DJdata[~nanRowsVel]
	ATdata = ATdata[~nanRowsVel]
	DTdata = DTdata[~nanRowsVel]

	if not learnGravComp:
		zeroRowsVel = (ArtificialVelData.T != 0).any()
		ArtificialVelData = ArtificialVelData[zeroRowsVel]
		AJdata = AJdata[zeroRowsVel]
		timeData = timeData[zeroRowsVel]
		DJdata = DJdata[zeroRowsVel]
		ATdata = ATdata[zeroRowsVel]
		DTdata = DTdata[zeroRowsVel]

	# replacing original with artifical velocity
	AJdata = pandas.concat((AJdata[jointAngleNames], ArtificialVelData[jointVelNames], AJdata[jointTorqueNames]), axis=1)
	AJdata.index = range(len(AJdata))
	DJdata.index = range(len(DJdata))
	ATdata.index = range(len(ATdata))
	DTdata.index = range(len(DTdata))
	timeData.index = range(len(timeData))

with open(AJacc, "w") as accCsv:
  pass

# Usual processing i.e. when joint velocity data is available
Jddot = np.concatenate((divByTime(AJdata.iloc[:,0+totalJoints].tolist()), divByTime(AJdata.iloc[:,1+totalJoints].tolist())), axis=1)

if(totalJoints > 2):
	jointIndex = 2
	while(jointIndex < totalJoints):
		Jddot = np.concatenate((Jddot, divByTime(AJdata.iloc[:,jointIndex+totalJoints].tolist())), axis=1)
		jointIndex = jointIndex + 1

Jddot[Jddot == -np.inf] = 0 # replacing inf with 0 for later removal
Jddot[Jddot == np.inf] = 0 # replacing inf with 0 for later removal

np.savetxt(AJacc, Jddot, delimiter=",")

AJaccData = pandas.read_csv(AJacc, names=jointAccNames)

# Removing the NaN rows to downsample the data
nanRows = AJaccData.T.isnull().any()
AJaccData = AJaccData[~nanRows]
AJdata = AJdata[~nanRows]
timeData = timeData[~nanRows]
DJdata = DJdata[~nanRows]  
ATdata = ATdata[~nanRows]
DTdata = DTdata[~nanRows]

# Removing duplicate entries. 
# Not done when learning the prediction of gravity compenastion torques because the recorded data points are purposely made to have ~ 0 vel and acc
if not learnGravComp:
	zeroRows = (AJaccData.T != 0).any()
	AJaccData = AJaccData[zeroRows]
	AJdata = AJdata[zeroRows]
	timeData = timeData[zeroRows]
	DJdata = DJdata[zeroRows]  
	ATdata = ATdata[zeroRows]
	DTdata = DTdata[zeroRows]

# The processed data
# The actual or real robot behaviour
processedActualData = pandas.concat((timeData[timeName], AJdata[jointAngleNames+jointVelNames], AJaccData[jointAccNames], AJdata[jointTorqueNames]), axis=1)
processedActualData.to_csv(PAD, index = False, header=True)

# The ideal or desired robot behaviour
processedDesiredData = pandas.concat((timeData[timeName], DJdata[jointAngleNames+jointVelNames]), axis=1)
processedDesiredData.to_csv(PDD, index = False, header=True)

# --------------------------------------------------------------

infoFile = os.path.join(savePath, "info" + ".txt") # Info about minimum points per block
info = open(infoFile, "w")

unsortedData = pandas.read_csv(PAD).iloc[:, 1:29]
rowCount = len(unsortedData)
blockData = pandas.DataFrame()

qResolutionList = []
qDotResolutionList = []

for jointIndex in range(totalJoints):
	qResolutionList.append(math.floor((qLimUpper[jointIndex]-qLimLower[jointIndex]-2*qMargin)/(2*qBound)))
	qDotResolutionList.append(math.floor((qDotLimUpper[jointIndex]-qDotLimLower[jointIndex]-2*qDotMargin)/(2*qDotBound)))

for jointIndex in range(totalJoints):
	info.write("joint: " + str(jointIndex + 1) + "\n") 	# Divisions of phase space along joint angle axis
	qResolution = int(qResolutionList[jointIndex])	# Divisions of phase space along joint velocity axis
	qDotResolution = int(qDotResolutionList[jointIndex]) # a list of lists containing the row indices of recorded data corresponding to the phase space block index they belong to

	info.write("phase space resolution: " + str(qResolution) + " x " + str(qDotResolution)  + "\n") 
	dataDistribList = [[] for i in range((qResolution*qDotResolution))]

	jData = unsortedData.iloc[:, jointIndex]  # respective joint angle data
	jDotData = unsortedData.iloc[:, totalJoints + jointIndex] # respective joint velocity data
	blockIndices = [] # indices of blocks corresponding to joint's data

	# Angle and velocity values which divide the phase space in resolution of (qResolution x qDotResolution)
	qSpace = np.linspace(qLimLower[jointIndex]+qMargin,
						 qLimUpper[jointIndex]-qMargin, num=qResolution+1)

	qDotSpace = np.linspace(-qDotLimLower[jointIndex]+qDotMargin,
							qDotLimUpper[jointIndex]-qDotMargin, num=qDotResolution+1)

	for rowIndex in range(rowCount):
		qTemp = jData[rowIndex]
		qDotTemp = jDotData[rowIndex]

		qBlockIndex = None
		qDotBlockIndex = None
			
		for qResolutionIndex in range(qResolution):
			if qResolutionIndex == 0:
				if qTemp < qSpace[qResolutionIndex+1]:
					qBlockIndex = qResolutionIndex
					break
			elif qResolutionIndex == qResolution - 1:
				if qTemp >= qSpace[qResolutionIndex]:
					qBlockIndex = qResolutionIndex
					break
			else:
				if qTemp >= qSpace[qResolutionIndex] and qTemp < qSpace[qResolutionIndex+1]:
					qBlockIndex = qResolutionIndex
					break

		for qDotResolutionIndex in range(qDotResolution):
			if qDotResolutionIndex == 0:
				if qDotTemp < qDotSpace[qDotResolutionIndex+1]:
					qDotBlockIndex = qDotResolutionIndex
					break
			elif qDotResolutionIndex == qDotResolution - 1:
				if qDotTemp >= qDotSpace[qDotResolutionIndex]:
					qDotBlockIndex = qDotResolutionIndex
					break
			else:
				if qDotTemp >= qDotSpace[qDotResolutionIndex] and qDotTemp < qDotSpace[qDotResolutionIndex+1]:
					qDotBlockIndex = qDotResolutionIndex
					break

		blockIndex = qResolution * qDotResolutionIndex + qResolutionIndex
		tempData = dataDistribList[blockIndex]
		tempData.append(rowIndex) 

		blockIndices.append(blockIndex)

	blockData.insert(jointIndex, jointAngleNames[jointIndex], pandas.Series(blockIndices))

	# dataDistribList completely filled for jointIndex
	# find the least number of data entries from among all the blocks
	minDataCount = rowCount
	for i in range(len(dataDistribList)):
		if len(dataDistribList[i]) < minDataCount:
			minDataCount = len(dataDistribList[i])

	info.write("data points per block: " + str(minDataCount) + "\n")
	info.write("cumulative data points: " + str(minDataCount*qResolution*qDotResolution) + "\n") 
	info.write("------------------------------" + "\n") 
	info.write("\n") 
	
	sortedList = []
	# randomly selecting the least number of entries from each block
	for i in range(len(dataDistribList)):
		sampledList = random.choices(dataDistribList[i], k=minDataCount)
		sortedList = sortedList + sampledList

blockIndicesFileName = "blockIndices"
blockIndicesFileName1 = os.path.join(savePath, blockIndicesFileName + ".csv")
blockData.to_csv(blockIndicesFileName1, index=False, header=True)