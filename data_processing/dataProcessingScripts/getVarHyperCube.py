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
from collections import Counter
from varNames import *


# def hyperCubeIndexHelper(index1, index2, index1RangeSize): # Index 1 along X, Index 2 along Y
# 	return index1RangeSize*index2 + index1 

# def getHyperCubeIndex(indices, indicesRange): # indices -> 1 x totalJoints; indicesRange -> [ [Joint 1 index range {0,1}], [Joint 2 index range] ..... ]
# 	hyperCubeIndex = indices[0]
# 	indexRangeSize = len(indicesRange[0])
# 	for jointIndex in range(1,len(indicesRange)):
# 		hyperCubeIndex = hyperCubeIndexHelper(hyperCubeIndex, indices[jointIndex], indexRangeSize)
# 		indexRangeSize = indexRangeSize*len(indicesRange[jointIndex])

# 	return hyperCubeIndex

rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

hyperCubeVarFileName = "hyperCubeVar"
hyperCubeVarFileName = os.path.join(savePath, hyperCubeVarFileName + ".csv")

rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)
jointData = rawData[:,1:1+(2*totalJoints)]

dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/eigSpreadBatchSize)

print("Calculating the cross-randomness")

indicesRange = [list(range(4)) for i in range(totalJoints)] # Joint angle has 2 indices - {0,1} (whether negative or positive), Joint velocity has 2 indices - {0,1} (whether negative or positive)
subCubeSize = 4**(totalJoints)

variance = []
hyperCubeIndices = []

for dataIndex in range(dataSize):
	print("Data entry: ", dataIndex, "/", dataSize)
	phaseIndices = [[] for i in range(totalJoints)]
	for jointIndex in range(totalJoints):
		qIndex = int(jointData[dataIndex, jointIndex] > 0)
		qDotIndex = int(jointData[dataIndex, jointIndex+totalJoints] > 0)
		phaseIndices[jointIndex] = 2*qDotIndex + qIndex 

	hyperCubeIndex = getHyperCubeIndex(phaseIndices, indicesRange)
	hyperCubeIndices = hyperCubeIndices + [hyperCubeIndex]

for iter in range(iterCount):
	print("Iteration count: ", iter, "/", iterCount)
	pointCounterSubCube = [0 for i in range(subCubeSize)] # Keeps a track of number of points in each sub-cube of the main 14 dimensional hypercube
	hIndices = hyperCubeIndices[0:(iter+1)*eigSpreadBatchSize]

	# Counting the number of entries in each sub-cube
	c = Counter(hIndices)
	for hIndexKey, repetitions in c.items():
		pointCounterSubCube[hIndexKey] = pointCounterSubCube[hIndexKey] + repetitions

	pointCounterSubCube = np.array(pointCounterSubCube)
	variance = variance + [[np.var(pointCounterSubCube)]]

# Writing the variances to csv
file = open(hyperCubeVarFileName, 'w+', newline ='')   
with file:     
	write = csv.writer(file)
	write.writerows(variance)