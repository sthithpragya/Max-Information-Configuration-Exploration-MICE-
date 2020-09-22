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

with open(r'param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

dataCollectionParamLoc = paramLoaded["dataCollectionParamLoc"]
with open(dataCollectionParamLoc,"r") as dataCollectionStream:
    dataCollectionParam = yaml.safe_load(dataCollectionStream)

savePath = paramLoaded["savePath"]
learnGravComp = paramLoaded["learnGravComp"]
predTorqueNames = paramLoaded["jointTorqueNames"]
for i in range(len(predTorqueNames)):
    predTorqueNames[i] = "pred" + predTorqueNames[i]


nonEqualisedDataSet = "processedActualData"

trainDataFileName = "trainData"

if not learnGravComp:
    testDataFileName = "testData"

# Train data mean and std
meanName = "trainDataMean"
stdName = "trainDataStd"
trainPredTauDataFileName = "InvDynPredTauData_train"

if not learnGravComp:
    testPredTauDataFileName = "InvDynPredTauData_test"

totalJoints = dataCollectionParam["totalJoints"]

resultDirectory = os.path.join(savePath, 'sparseFormat') 
if not os.path.exists(resultDirectory):
    os.makedirs(resultDirectory)


trainPredTauDataFileName = os.path.join(savePath, trainPredTauDataFileName + ".csv")
trainPredTauData = pandas.read_csv(trainPredTauDataFileName, names=predTorqueNames,index_col=False)

if not learnGravComp:
    testPredTauDataFileName = os.path.join(savePath, testPredTauDataFileName + ".csv")
    testPredTauData = pandas.read_csv(testPredTauDataFileName, names=predTorqueNames,index_col=False)


meanFileName = os.path.join(resultDirectory, meanName + ".txt")
stdFileName = os.path.join(resultDirectory, stdName + ".txt")

meanFileName2 = os.path.join(savePath, meanName + ".txt")
stdFileName2 = os.path.join(savePath, stdName + ".txt")

trainDataFileName = os.path.join(savePath, trainDataFileName + ".csv")
if not learnGravComp:
    testDataFileName = os.path.join(savePath, testDataFileName + ".csv")

trainData = pandas.read_csv(trainDataFileName)
if not learnGravComp:
    testData = pandas.read_csv(testDataFileName)

trainDataSupplemented = pandas.concat([trainData.iloc[:, 1:(4*totalJoints)+1], trainPredTauData], axis=1) # Train (q, qdot, qddot, tau, predTau) 

if not learnGravComp:
    testDataSupplemented = pandas.concat([testData.iloc[:, 1:(4*totalJoints)+1], testPredTauData], axis=1) # Test (q, qdot, qddot, tau, predTau)

jointTrainData = trainDataSupplemented.iloc[:, 0:3*totalJoints]
tauTrainData = -trainDataSupplemented.iloc[:, 3*totalJoints:4*totalJoints] # - sign because of the way kuka iiwa records torque data
predTauTrainData = -trainDataSupplemented.iloc[:, 4*totalJoints:5*totalJoints] # - sign because of the way kuka iiwa records torque data
errorTrainData = tauTrainData - predTauTrainData.values


if not learnGravComp:
    jointTestData = testDataSupplemented.iloc[:, 0:3*totalJoints]
    tauTestData = -testDataSupplemented.iloc[:, 3*totalJoints:4*totalJoints]
    predTauTestData = -testDataSupplemented.iloc[:, 4*totalJoints:5*totalJoints]
    errorTestData = tauTestData - predTauTestData.values

dataMean = jointTrainData.mean()
print("mean of the joint train data")
print(dataMean)
dataStd = jointTrainData.std()
print("std of the joint train data")
print(dataStd)

np.savetxt(meanFileName, dataMean.values, fmt='%1.3f')
np.savetxt(stdFileName, dataStd.values, fmt='%1.3f')

np.savetxt(meanFileName2, dataMean.values, fmt='%1.3f')
np.savetxt(stdFileName2, dataStd.values, fmt='%1.3f')

scaledTrainData = (jointTrainData - dataMean)/dataStd

if not learnGravComp:
    scaledTestData = (jointTestData - dataMean)/dataStd


#saves training and testing data in sparse formats for each model
for modelIndex in range(totalJoints):

    sparseTrainDataFileName = "sparseTrainData" + str(modelIndex)
    sparseTrainDataFileName = os.path.join(resultDirectory, sparseTrainDataFileName + ".dat")
    dump_svmlight_file(scaledTrainData, tauTrainData.iloc[:,modelIndex], sparseTrainDataFileName, zero_based=False)

    sparseErrorTrainDataFileName = "sparseErrorTrainData" + str(modelIndex)
    sparseErrorTrainDataFileName = os.path.join(resultDirectory, sparseErrorTrainDataFileName + ".dat")
    dump_svmlight_file(scaledTrainData, errorTrainData.iloc[:,modelIndex], sparseErrorTrainDataFileName, zero_based=False)

    if not learnGravComp:
        sparseTestDataFileName = "sparseTestData" + str(modelIndex)
        sparseTestDataFileName = os.path.join(resultDirectory, sparseTestDataFileName + ".dat")
        dump_svmlight_file(scaledTestData, tauTestData.iloc[:,modelIndex], sparseTestDataFileName, zero_based=False)

        sparseErrorTestDataFileName = "sparseErrorTestData" + str(modelIndex)
        sparseErrorTestDataFileName = os.path.join(resultDirectory, sparseErrorTestDataFileName + ".dat")
        dump_svmlight_file(scaledTestData, errorTestData.iloc[:,modelIndex], sparseErrorTestDataFileName, zero_based=False)

normalisedTrainData = np.concatenate((scaledTrainData.to_numpy(),tauTrainData.to_numpy()),axis=1)

if not learnGravComp:
    normalisedTestData = np.concatenate((scaledTestData.to_numpy(),tauTestData.to_numpy()),axis=1)

normalisedErrorTrainData = np.concatenate((scaledTrainData.to_numpy(),errorTrainData.to_numpy()),axis=1)

if not learnGravComp:
    normalisedErrorTestData = np.concatenate((scaledTestData.to_numpy(),errorTestData.to_numpy()),axis=1)

normalisedPredTrainData = np.concatenate((scaledTrainData.to_numpy(),predTauTrainData.to_numpy()),axis=1)

if not learnGravComp:
    normalisedPredTestData = np.concatenate((scaledTestData.to_numpy(),predTauTestData.to_numpy()),axis=1)


np.savetxt(os.path.join(savePath, "normalisedTrainData.csv"), normalisedTrainData, delimiter=",")

if not learnGravComp:
    np.savetxt(os.path.join(savePath, "normalisedTestData.csv"), normalisedTestData, delimiter=",")

np.savetxt(os.path.join(savePath, "normalisedErrorTrainData.csv"), normalisedErrorTrainData, delimiter=",")

if not learnGravComp:
    np.savetxt(os.path.join(savePath, "normalisedErrorTestData.csv"), normalisedErrorTestData, delimiter=",")

np.savetxt(os.path.join(savePath, "normalisedPredTrainData.csv"), normalisedPredTrainData, delimiter=",")
    
if not learnGravComp:
    np.savetxt(os.path.join(savePath, "normalisedPredTestData.csv"), normalisedPredTestData, delimiter=",")

