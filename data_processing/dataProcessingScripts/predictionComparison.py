#!/usr/bin/env python
import numpy as np
import csv
import os.path
import pandas
import math
import yaml
import io
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from varNames import *
import copy


sensorDataFileName = "testTauData_filtered"
URDF_predictionFileName = "InvDynPredTauData_test"
nuSVR_Full_PredictionFileName = "nuSVR_Full_predictions_test"
ANN_Full_PredictionFileName = "ANN_Full_predictions_test"
ANN_Error_PredictionFileName = "ANN_Error_predictions_test"

sensorDataFileName = os.path.join(savePath, sensorDataFileName + ".csv")
URDF_predictionFileName = os.path.join(savePath, URDF_predictionFileName + ".csv")
ANN_Full_PredictionFileName = os.path.join(savePath, ANN_Full_PredictionFileName + ".csv")
ANN_Error_PredictionFileName = os.path.join(savePath, ANN_Error_PredictionFileName + ".csv")

colList = tuple(list(range(totalJoints)))

sensorData = np.genfromtxt(sensorDataFileName, delimiter=',', usecols=colList, dtype=float)
URDF_prediction = np.genfromtxt(URDF_predictionFileName, delimiter=',', usecols=colList, dtype=float)
ANN_Full_Prediction = np.genfromtxt(ANN_Full_PredictionFileName, delimiter=',', usecols=colList, dtype=float)
ANN_Error_Prediction = np.genfromtxt(ANN_Error_PredictionFileName, delimiter=',', usecols=colList, dtype=float)

nuSVR_Full_Prediction = None

for jointIndex in range(totalJoints):
	tempName = nuSVR_Full_PredictionFileName
	tempName = os.path.join(savePath, tempName + str(jointIndex) + ".csv")
	tempData = np.genfromtxt(tempName, delimiter=',', dtype=float)
	tempData = np.reshape(tempData, (-1,1))
	
	if jointIndex == 0:
		nuSVR_Full_Prediction = copy.deepcopy(tempData)
	else:
		nuSVR_Full_Prediction = np.concatenate((nuSVR_Full_Prediction, tempData), axis=1)

if recordingResultantTorque:
	sensorData = -sensorData
	URDF_prediction = -URDF_prediction

ANN_Error_Prediction = ANN_Error_Prediction - URDF_prediction

FB_ANN_Full = np.abs(sensorData - ANN_Full_Prediction)
FB_ANN_Error = np.abs(sensorData - ANN_Error_Prediction)
FB_nuSVR_Full = np.abs(sensorData - nuSVR_Full_Prediction)
FB_URDF = np.abs(sensorData - URDF_prediction)


ANN_Full_mean = [[] for i in range(totalJoints)]
ANN_Error_mean = [[] for i in range(totalJoints)]
nuSVR_Full_mean = [[] for i in range(totalJoints)]
URDF_mean = [[] for i in range(totalJoints)]

ANN_Full_std = [[] for i in range(totalJoints)]
ANN_Error_std = [[] for i in range(totalJoints)]
nuSVR_Full_std = [[] for i in range(totalJoints)]
URDF_std = [[] for i in range(totalJoints)]

sensorData = np.abs(sensorData)
infoFile = os.path.join(savePath, "modelComparison" + ".txt") # Info about minimum points per block
info = open(infoFile, "w")

if testMode == "static":

	for jointIndex in range(totalJoints):
		sensor_Val = pandas.Series(sensorData[:,jointIndex])
		ANN_Full_val = pandas.Series(FB_ANN_Full[:, jointIndex])
		ANN_Error_val = pandas.Series(FB_ANN_Error[:, jointIndex])
		nuSVR_Full_val = pandas.Series(FB_nuSVR_Full[:, jointIndex])
		URDF_val = pandas.Series(FB_URDF[:, jointIndex])

		validRows = (sensor_Val > 0.5*sensor_Val.quantile(q=0.9))

		sensor_Val = sensor_Val[validRows]
		ANN_Full_val = ANN_Full_val[validRows]
		ANN_Error_val = ANN_Error_val[validRows]
		nuSVR_Full_val = nuSVR_Full_val[validRows]
		URDF_val = URDF_val[validRows]

		ANN_Full_val = ANN_Full_val/sensor_Val
		ANN_Error_val = ANN_Error_val/sensor_Val
		nuSVR_Full_val = nuSVR_Full_val/sensor_Val
		URDF_val = URDF_val/sensor_Val

		ANN_Full_mean[jointIndex] = ANN_Full_val.mean() 
		ANN_Error_mean[jointIndex] = ANN_Error_val.mean() 
		nuSVR_Full_mean[jointIndex] = nuSVR_Full_val.mean() 
		URDF_mean[jointIndex] = URDF_val.mean() 

		ANN_Full_std[jointIndex] = ANN_Full_val.std() 
		ANN_Error_std[jointIndex] = ANN_Error_val.std() 
		nuSVR_Full_std[jointIndex] = nuSVR_Full_val.std() 
		URDF_std[jointIndex] = URDF_val.std() 

	info.write("FB Mean" + "\n") 	
	info.write("ANN_Full 	:" + str(ANN_Full_mean) + "\n")
	info.write("ANN_Error 	:" + str(ANN_Error_mean) + "\n")
	info.write("nuSVR_Full 	:" + str(nuSVR_Full_mean) + "\n")
	info.write("URDF 		:" + str(URDF_mean) + "\n")

	info.write("\n") 	

	info.write("FB Std" + "\n") 	
	info.write("ANN_Full 	:" + str(ANN_Full_std) + "\n")
	info.write("ANN_Error 	:" + str(ANN_Error_std) + "\n")
	info.write("nuSVR_Full 	:" + str(nuSVR_Full_std) + "\n")
	info.write("URDF 		:" + str(URDF_std) + "\n")

else:

	sensorData = pandas.DataFrame(sensorData)
	FB_ANN_Full = pandas.DataFrame(FB_ANN_Full)
	FB_ANN_Error = pandas.DataFrame(FB_ANN_Error)
	FB_nuSVR_Full = pandas.DataFrame(FB_nuSVR_Full)
	FB_URDF = pandas.DataFrame(FB_URDF)

	validRows = (sensorData.T > 0).all()

	sensorData = sensorData[validRows]
	FB_ANN_Full = FB_ANN_Full[validRows]
	FB_ANN_Error = FB_ANN_Error[validRows]
	FB_nuSVR_Full = FB_nuSVR_Full[validRows]
	FB_URDF = FB_URDF[validRows]

	FB_ANN_Full = FB_ANN_Full/sensorData
	FB_ANN_Error = FB_ANN_Error/sensorData
	FB_nuSVR_Full = FB_nuSVR_Full/sensorData
	FB_URDF = FB_URDF/sensorData

	info.write("FB Mean" + "\n") 	

	info.write("ANN_Full 	:" + str(FB_ANN_Full.mean(axis=0).tolist()) + "\n")
	info.write("ANN_Error 	:" + str(FB_ANN_Error.mean(axis=0).tolist()) + "\n")
	info.write("nuSVR_Full 	:" + str(FB_nuSVR_Full.mean(axis=0).tolist()) + "\n")
	info.write("URDF 		:" + str(FB_URDF.mean(axis=0).tolist()) + "\n")



	# info.write("ANN_Full 	:" + str(np.mean(FB_ANN_Full,axis=0).tolist()) + "\n")
	# info.write("ANN_Error 	:" + str(np.mean(FB_ANN_Error,axis=0).tolist()) + "\n")
	# info.write("nuSVR_Full 	:" + str(np.mean(FB_nuSVR_Full,axis=0).tolist()) + "\n")
	# info.write("URDF 		:" + str(np.mean(FB_URDF,axis=0).tolist()) + "\n")

	info.write("FB Std" + "\n") 	
	info.write("ANN_Full 	:" + str(FB_ANN_Full.std(axis=0).tolist()) + "\n")
	info.write("ANN_Error 	:" + str(FB_ANN_Error.std(axis=0).tolist()) + "\n")
	info.write("nuSVR_Full 	:" + str(FB_nuSVR_Full.std(axis=0).tolist()) + "\n")
	info.write("URDF 		:" + str(FB_URDF.std(axis=0).tolist()) + "\n")









# info.write("\n") 	

# info.write("FB MSE" + "\n") 	
# info.write("ANN_Full 	:" + str(np.mean(np.square(FB_ANN_Full),axis=0)) + "\n")
# info.write("ANN_Error 	:" + str(np.mean(np.square(FB_ANN_Error),axis=0)) + "\n")
# info.write("nuSVR_Full 	:" + str(np.mean(np.square(FB_nuSVR_Full),axis=0)) + "\n")
# info.write("URDF 		:" + str(np.mean(np.square(FB_URDF),axis=0)) + "\n")