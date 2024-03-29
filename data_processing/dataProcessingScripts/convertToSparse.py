#!/usr/bin/env python
import csv
import os.path
import pandas
import math
import numpy as np
import yaml
import io
from sklearn import preprocessing
from sklearn import svm
from sklearn.svm import SVR
from sklearn.model_selection import train_test_split
from joblib import dump, load
from sklearn.model_selection import GridSearchCV
from sklearn.datasets import dump_svmlight_file

from varNames import *

print("CONVERTING TRAIN AND TEST SETS TO SPARSE FORMAT")

nonEqualisedDataSet = "processedActualData"

trainDataFileName = "trainData"
testDataFileName = "testData"

# Train data mean and std
meanName = "trainDataMean"
stdName = "trainDataStd"
trainPredTauDataFileName = "InvDynPredTauData_train"
testPredTauDataFileName = "InvDynPredTauData_test"

resultDirectory = os.path.join(savePath, 'sparseFormat') 
if not os.path.exists(resultDirectory):
	os.makedirs(resultDirectory)

trainPredTauDataFileName = os.path.join(savePath, trainPredTauDataFileName + ".csv")
trainPredTauData = pandas.read_csv(trainPredTauDataFileName, names=predTorqueNames,index_col=False)

testPredTauDataFileName = os.path.join(savePath, testPredTauDataFileName + ".csv")
testPredTauData = pandas.read_csv(testPredTauDataFileName, names=predTorqueNames,index_col=False)


meanFileName = os.path.join(resultDirectory, meanName + ".txt")
stdFileName = os.path.join(resultDirectory, stdName + ".txt")

meanFileName2 = os.path.join(savePath, meanName + ".txt")
stdFileName2 = os.path.join(savePath, stdName + ".txt")

trainDataFileName = os.path.join(savePath, trainDataFileName + ".csv")
testDataFileName = os.path.join(savePath, testDataFileName + ".csv")



if not useForTesting:

	trainData = pandas.read_csv(trainDataFileName)
	trainDataSupplemented = pandas.concat([trainData.iloc[:, 1:(4*totalJoints)+1], trainPredTauData], axis=1) # Train (q, qdot, qddot, tau, predTau) 
	jointTrainData = trainDataSupplemented.iloc[:, 0:3*totalJoints]
	tauTrainData = trainDataSupplemented.iloc[:, 3*totalJoints:4*totalJoints] 
	predTauTrainData = trainDataSupplemented.iloc[:, 4*totalJoints:5*totalJoints] 

	# - sign because of the way kuka iiwa records torque data
	if recordingResultantTorque:
		tauTrainData = - tauTrainData
		predTauTrainData = - predTauTrainData

	# errorTrainData = tauTrainData - predTauTrainData.values # Original
	errorTrainData = tauTrainData + predTauTrainData.values   # Changed on 13.10

	dataMean = jointTrainData.mean()
	print("> Mean of the joint train data")
	print(dataMean)
	dataStd = jointTrainData.std()
	print("> Std of the joint train data")
	print(dataStd)

	np.savetxt(meanFileName, dataMean.values, fmt='%1.3f')
	np.savetxt(stdFileName, dataStd.values, fmt='%1.3f')

	np.savetxt(meanFileName2, dataMean.values, fmt='%1.3f')
	np.savetxt(stdFileName2, dataStd.values, fmt='%1.3f')

	scaledTrainData = (jointTrainData - dataMean)/dataStd



testData = pandas.read_csv(testDataFileName)
testDataSupplemented = pandas.concat([testData.iloc[:, 1:(4*totalJoints)+1], testPredTauData], axis=1) # Test (q, qdot, qddot, tau, predTau)
jointTestData = testDataSupplemented.iloc[:, 0:3*totalJoints]
tauTestData = testDataSupplemented.iloc[:, 3*totalJoints:4*totalJoints]
predTauTestData = testDataSupplemented.iloc[:, 4*totalJoints:5*totalJoints]

if recordingResultantTorque:
	tauTestData = - tauTestData
	predTauTestData = - predTauTestData

# errorTestData = tauTestData - predTauTestData.values
errorTestData = tauTestData + predTauTestData.values

if not useForTesting:
	scaledTestData = (jointTestData - dataMean)/dataStd
else:
	scaledTestData = jointTestData

#saves training and testing data in sparse formats for each model
print("> Saving in sparse format")
for modelIndex in range(totalJoints):

	if not useForTesting:
		sparseTrainDataFileName = "sparseTrainData" + str(modelIndex)
		sparseTrainDataFileName = os.path.join(resultDirectory, sparseTrainDataFileName + ".dat")
		dump_svmlight_file(scaledTrainData, tauTrainData.iloc[:,modelIndex], sparseTrainDataFileName, zero_based=False)

		sparseErrorTrainDataFileName = "sparseErrorTrainData" + str(modelIndex)
		sparseErrorTrainDataFileName = os.path.join(resultDirectory, sparseErrorTrainDataFileName + ".dat")
		dump_svmlight_file(scaledTrainData, errorTrainData.iloc[:,modelIndex], sparseErrorTrainDataFileName, zero_based=False)

		sparseTestDataFileName = "sparseTestData" + str(modelIndex)
		sparseTestDataFileName = os.path.join(resultDirectory, sparseTestDataFileName + ".dat")
		dump_svmlight_file(scaledTestData, tauTestData.iloc[:,modelIndex], sparseTestDataFileName, zero_based=False)

		sparseErrorTestDataFileName = "sparseErrorTestData" + str(modelIndex)
		sparseErrorTestDataFileName = os.path.join(resultDirectory, sparseErrorTestDataFileName + ".dat")
		dump_svmlight_file(scaledTestData, errorTestData.iloc[:,modelIndex], sparseErrorTestDataFileName, zero_based=False)

print("> Saving in csv format")

if not useForTesting:
	normalisedTrainData = np.concatenate((scaledTrainData.to_numpy(),tauTrainData.to_numpy()),axis=1)
	normalisedErrorTrainData = np.concatenate((scaledTrainData.to_numpy(),errorTrainData.to_numpy()),axis=1)
	np.savetxt(os.path.join(savePath, "normalisedTrainData.csv"), normalisedTrainData, delimiter=",")
	np.savetxt(os.path.join(savePath, "normalisedErrorTrainData.csv"), normalisedErrorTrainData, delimiter=",")
	
	normalisedTestData = np.concatenate((scaledTestData.to_numpy(),tauTestData.to_numpy()),axis=1)
	normalisedErrorTestData = np.concatenate((scaledTestData.to_numpy(),errorTestData.to_numpy()),axis=1)
	np.savetxt(os.path.join(savePath, "normalisedTestData.csv"), normalisedTestData, delimiter=",")
	np.savetxt(os.path.join(savePath, "normalisedErrorTestData.csv"), normalisedErrorTestData, delimiter=",")