import numpy as np
import csv
import os.path
import pandas
import math
import numpy as np
import yaml
import io
import scipy
import random
import itertools

from varNames import *

rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

eigSpreadFileName = "eigSpread"
eigSpreadFileName = os.path.join(savePath, eigSpreadFileName + ".csv")

rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)
jointData = rawData[:,1:1+(2*totalJoints)]

eigenValSpread = []
dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/eigSpreadBatchSize)

print("Calculating the cross-randomness")

for iter in range(iterCount):
	print("Iteration count: ", iter, "/", iterCount)
	tempData = jointData[0:(iter+1)*eigSpreadBatchSize,:]
	print(np.shape(tempData))
	dataMean = np.mean(tempData,axis=0)
	dataStd = np.std(tempData,axis=0)

	tempData = (tempData - dataMean)/dataStd # normalised data

	covarMatrix = np.dot(np.transpose(tempData), tempData)/((iter+1)*eigSpreadBatchSize)
	eigVal, eigvec = np.linalg.eig(covarMatrix)

	eigenValSpread = eigenValSpread + [[np.var(eigVal)]]

file = open(eigSpreadFileName, 'w+', newline ='')   
with file:     
	write = csv.writer(file)
	write.writerows(eigenValSpread)