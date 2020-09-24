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
import copy
from varNames import *

eigSpreadBaseName = "eigSpread_blockIndices"

rawDataFileName = "blockIndices"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=int, skip_header=1)

dataSize =  np.shape(rawData)[0]
iterCount = int(dataSize/eigSpreadBatchSize)

gridSize = [[] for i in range(totalJoints)]

for jointIndex in range(totalJoints):
	gridSize[jointIndex] = qResolutionList[jointIndex]*qDotResolutionList[jointIndex] ## getting the total number of modules in each phase space grid

maxVal = np.array(gridSize)-1

print("Calculating the cross-randomness")

for baseIter in range(11):
	jointData = copy.deepcopy(rawData)

	eigSpreadFileName = os.path.join(savePath, eigSpreadBaseName + "_" + str(baseIter) + ".csv")

	if baseIter > 0:

		# Default indices allocated in ascending order [0,gridSize)
		for jointIndex in range(totalJoints):
			random.seed(10*baseIter+totalJoints)
			newIndices = random.sample(range(0,gridSize[jointIndex]),gridSize[jointIndex]) # Generating new indices for the modules in the grids randomly bewteen 0 and whatever the size is
			newIndices = np.array(newIndices)

			jointData[:,jointIndex] = newIndices[jointData[:,jointIndex].tolist()]


		# Reindexed data ready
	eigenValSpread = []

	for iter in range(iterCount):
		print("Sequence: ", baseIter+1, "/", 10, "| Iteration count: ", iter+1, "/", iterCount)
		tempData = jointData[0:(iter+1)*eigSpreadBatchSize,:]
		print(np.shape(tempData))
		
		# Min-Max scaling
		tempData = tempData/maxVal

		# Centering		
		dataMean = np.mean(tempData,axis=0)

		tempData = (tempData - dataMean) # normalised data

		covarMatrix = np.dot(np.transpose(tempData), tempData)/((iter+1)*eigSpreadBatchSize)
		eigVal, eigvec = np.linalg.eig(covarMatrix)

		eigenValSpread = eigenValSpread + [[np.var(eigVal)]]

	file = open(eigSpreadFileName, 'w+', newline ='')   
	with file:     
		write = csv.writer(file)
		write.writerows(eigenValSpread)