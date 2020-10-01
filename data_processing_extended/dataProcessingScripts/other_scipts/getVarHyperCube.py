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

rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

hyperCubeVarFileName = "hyperCubeVar"
hyperCubeVarFileName = os.path.join(savePath, hyperCubeVarFileName + ".csv")

hyperCubeIndicesFileName = "hyperCubeIndices"
hyperCubeIndicesFileName = os.path.join(savePath, hyperCubeIndicesFileName + ".csv")


rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)
jointData = rawData[:,1:1+(2*totalJoints)]

dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/entropyBatchSize)

print("Calculating the cross-randomness")

indicesRange = [list(range(4)) for i in range(totalJoints)] # Joint angle has 2 indices - {0,1} (whether negative or positive), Joint velocity has 2 indices - {0,1} (whether negative or positive)
# Combined we have 4 indices {0,1,2,3}
subCubeSize = 4**(totalJoints)

variance = []
hyperCubeIndices = [[] for i in range(dataSize)]
hyperCubeIndicesToFile = [[] for i in range(dataSize)]

file1 = open(hyperCubeIndicesFileName, 'w+', newline ='')   

# Calculating the indices
for dataIndex in range(dataSize):
	print("Data entry: ", dataIndex, "/", dataSize)
	phaseIndices = [[] for i in range(totalJoints)]

	for jointIndex in range(totalJoints):
		qIndex = int(jointData[dataIndex, jointIndex] > 0)
		qDotIndex = int(jointData[dataIndex, jointIndex+totalJoints] > 0)
		phaseIndices[jointIndex] = 2*qDotIndex + qIndex 

	hyperCubeIndex = getHyperCubeIndex(phaseIndices, indicesRange)
	hyperCubeIndices[dataIndex] = hyperCubeIndex
	hyperCubeIndicesToFile[dataIndex] = [hyperCubeIndex]

# Writing the indices to csv
with file1:     
	write = csv.writer(file1)
	write.writerows(hyperCubeIndicesToFile)

# Calculating the variances
for iter in range(iterCount):
	print("Iteration count: ", iter, "/", iterCount)
	pointCounterSubCube = [0 for i in range(subCubeSize)] # Keeps a track of number of points in each sub-cube of the main 14 dimensional hypercube
	hIndices = hyperCubeIndices[0:(iter+1)*entropyBatchSize]

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