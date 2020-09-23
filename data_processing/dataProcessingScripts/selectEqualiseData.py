import numpy as np
import csv
import os.path
import pandas
import math
import yaml
import io
import scipy
import random
import itertools
from numpy import genfromtxt

from varNames import *

# Entire data
sortedIndicesFileName = "sortedIndices"
rawDataFileName = "processedActualData"
blockIndicesFileName = "blockIndices"
velocityDistribFileName = "velocityDistribution"
origVelocityDistribFileName = "rawVelocityDistribution"

sortedIndicesFileName = os.path.join(savePath, sortedIndicesFileName + ".csv")
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")
blockIndicesFileName = os.path.join(savePath, blockIndicesFileName + ".csv")
velocityDistribFileName = os.path.join(savePath, velocityDistribFileName + ".txt")
origVelocityDistribFileName = os.path.join(savePath, origVelocityDistribFileName + ".txt")

# Sub-sampled data as Train and cross-validation sets
trainDataFileName = "trainData"
trainJointDataFileName = "trainJointData"  
filteredTrainJointDataFileName = "trainJointData_filtered"
filteredTrainTauFileName = "trainTauData_filtered"

trainDataFileName = os.path.join(savePath, trainDataFileName + ".csv")
trainJointDataFileName = os.path.join(savePath, trainJointDataFileName + ".csv")
filteredTrainJointDataFileName = os.path.join(savePath, filteredTrainJointDataFileName + ".csv")
filteredTrainTauFileName = os.path.join(savePath, filteredTrainTauFileName + ".csv")

# Remaining data as Test Set
testDataFileName = "testData"
testJointDataFileName = "testJointData"
filteredTestJointDataFileName = "testJointData_filtered"
filteredTestTauFileName = "testTauData_filtered"  

testDataFileName = os.path.join(savePath, testDataFileName + ".csv")
testJointDataFileName = os.path.join(savePath, testJointDataFileName + ".csv")
filteredTestJointDataFileName = os.path.join(savePath, filteredTestJointDataFileName + ".csv")
filteredTestTauFileName = os.path.join(savePath, filteredTestTauFileName + ".csv")

## Filtering
# Complete data
rawData = pandas.read_csv(rawDataFileName)
tauData = rawData[rawData.columns[22:22+totalJoints]]
jointData = rawData[rawData.columns[1:1+3*totalJoints]]
rawFilteredTau = []
rawFilteredQ = []
rawFilteredQDot = []
rawFilteredQDDot = []

# Filtering (or not) the q, qdot, qddot, tau values 
for jointIndex in range(totalJoints):
    if toFilterTau:
        rawFilteredTau = rawFilteredTau + [tauData.iloc[:,jointIndex].rolling(window=tauFilterVal, center=True, min_periods=1).mean().tolist()]
    else:
        rawFilteredTau = rawFilteredTau + [tauData.iloc[:,jointIndex].tolist()]

    if toFilterQ:
        rawFilteredQ = rawFilteredQ + [jointData.iloc[:,jointIndex].rolling(window=qFilterVal, center=True, min_periods=1).mean().tolist()]
    else:
        rawFilteredQ = rawFilteredQ + [jointData.iloc[:,jointIndex].tolist()]

    if toFilterQDot:
        filteredVelocity = jointData.iloc[:,totalJoints+jointIndex].rolling(window=qDotFilterVal, center=True, min_periods=1).mean().tolist()
    else:
        filteredVelocity = jointData.iloc[:,totalJoints+jointIndex].tolist()

    rawFilteredQDot = rawFilteredQDot + [filteredVelocity]
    filteredVelArray = np.array(filteredVelocity + [filteredVelocity[-1]])
    filteredAcc = (np.diff(filteredVelArray)/dt).tolist()
    rawFilteredQDDot = rawFilteredQDDot + [filteredAcc]

rawFilteredTau = pandas.DataFrame.from_records(rawFilteredTau).transpose()
rawFilteredQ = pandas.DataFrame.from_records(rawFilteredQ).transpose()
rawFilteredQDot = pandas.DataFrame.from_records(rawFilteredQDot).transpose()
rawFilteredQDDot = pandas.DataFrame.from_records(rawFilteredQDDot).transpose()
rawFilteredJointData = pandas.concat([rawFilteredQ, rawFilteredQDot, rawFilteredQDDot], axis=1)


# Splitting the data (into train and test) and saving

sortedRows = genfromtxt(sortedIndicesFileName, delimiter=',').astype(int) # Data points' indices arranged wrt relevance scores in descending order
trainRows = [] # indices of sub-sampled data to be used for training and cross-validation
testRows = [] # indices of the remainder data

trainRows = sortedRows[0:bestPointCount].tolist() 
remainingEntries = sortedRows[bestPointCount:]
random.shuffle(remainingEntries)

trainRows = trainRows + remainingEntries[:randomPointCount].tolist()
testRows = remainingEntries[randomPointCount:]

#Saving the train data
trainData = rawData.iloc[trainRows]
trainTauData_filtered = rawFilteredTau.iloc[trainRows]
trainJointData_filtered = rawFilteredJointData.iloc[trainRows]
trainTauData_filtered = pandas.concat([trainTauData_filtered, trainData.time], axis=1)
trainJointData_filtered = pandas.concat([trainJointData_filtered, trainData.time], axis=1)

trainData = trainData.sort_values('time')
trainTauData_filtered = trainTauData_filtered.sort_values('time')
trainJointData_filtered = trainJointData_filtered.sort_values('time')
del trainTauData_filtered['time']
del trainJointData_filtered['time']

trainData.to_csv(trainDataFileName, index=False, header=True) # complete subsampled data [time, q, qdot, qddot, tau, gtau , ee_X, ee_Y, ee_Z]
trainTauData_filtered.to_csv(filteredTrainTauFileName, index=False, header=False) # Filtered [Tau1 Tau2 .... Tau7]
trainJointData_filtered.to_csv(filteredTrainJointDataFileName, index=False, header=False) # Filtered [q, qdot, qddot]

trainJointData = trainData.iloc[:, 1:(1+3*totalJoints)] # only [q, qdot, qddot]
trainJointData.to_csv(trainJointDataFileName, index=False, header=False)

#Saving the test data
testData = rawData.iloc[testRows]
testTauData_filtered = rawFilteredTau.iloc[testRows]
testJointData_filtered = rawFilteredJointData.iloc[testRows]
testTauData_filtered = pandas.concat([testTauData_filtered, testData.time], axis=1)
testJointData_filtered = pandas.concat([testJointData_filtered, testData.time], axis=1)

testData = testData.sort_values('time')
testTauData_filtered = testTauData_filtered.sort_values('time')
testJointData_filtered = testJointData_filtered.sort_values('time')
del testTauData_filtered['time']
del testJointData_filtered['time']

testData.to_csv(testDataFileName, index=False, header=True) # complete subsampled data [time, q, qdot, qddot, tau, gtau , ee_X, ee_Y, ee_Z]
testTauData_filtered.to_csv(filteredTestTauFileName, index=False, header=False) # Filtered [Tau1 Tau2 .... Tau7]
testJointData_filtered.to_csv(filteredTestJointDataFileName, index=False, header=False) # Filtered [q, qdot, qddot]

testJointData = testData.iloc[:, 1:(1+3*totalJoints)] # only [q, qdot, qddot]
testJointData.to_csv(testJointDataFileName, index=False, header=False)



with open(velocityDistribFileName, 'w+') as velocityDistribFile:
    # Information about the distribution of sub-sampled data
    velocityDistribFile.writelines(
        "number of total data points " + str(len(rawData)) + "\n")
    velocityDistribFile.writelines(
        "number of selected data points " + str(len(trainData)) + "\n")

    for jointIndex in range(totalJoints):
        highSpeedLimUpper = qDotLimUpper[jointIndex]/2.0
        highSpeedLimLower = qDotLimLower[jointIndex]/2.0
        
        velData = trainData.iloc[:, 1+totalJoints+jointIndex]

        lowSpeedDataCount = len(velData[velData.between(highSpeedLimLower, highSpeedLimUpper)])
        highSpeedDataCount = len(velData[~velData.between(highSpeedLimLower, highSpeedLimUpper)])

        velocityDistribFile.writelines(
            "\n" + "point count for joint " + str(jointIndex+1) + "\n")
        velocityDistribFile.writelines(
            "low speed point count:" + str(lowSpeedDataCount) + "\n")
        velocityDistribFile.writelines(
            "high speed point count:" + str(highSpeedDataCount) + "\n")

with open(origVelocityDistribFileName, 'w+') as origvelocityDistribFile:
    # Information about the distribution of original (and processed) actual data
    origvelocityDistribFile.writelines(
        "number of total data points " + str(len(rawData)) + "\n")

    for jointIndex in range(totalJoints):
        highSpeedLimUpper = qDotLimUpper[jointIndex]/2.0
        highSpeedLimLower = qDotLimLower[jointIndex]/2.0

        rawVelData = rawData.iloc[:, 1+totalJoints+jointIndex]

        rawlowSpeedDataCount = len(rawVelData[rawVelData.between(highSpeedLimLower, highSpeedLimUpper)])
        rawhighSpeedDataCount = len(rawVelData[~rawVelData.between(highSpeedLimLower, highSpeedLimUpper)])

        origvelocityDistribFile.writelines(
            "\n" + "point count for joint " + str(jointIndex+1) + "\n")
        origvelocityDistribFile.writelines(
            "low speed point count:" + str(rawlowSpeedDataCount) + "\n")
        origvelocityDistribFile.writelines(
            "high speed point count:" + str(rawhighSpeedDataCount) + "\n")

print("Prepared Train and Test sets")