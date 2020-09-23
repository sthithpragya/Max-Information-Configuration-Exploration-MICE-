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
rawData = pandas.read_csv(rawDataFileName)

blockIndicesFileName = "blockIndices"
blockIndicesFileName = os.path.join(savePath, blockIndicesFileName + ".csv")
blockData = pandas.read_csv(blockIndicesFileName) # data at each time instance segregated in joint-respective blocks 

sortedIndicesFileName = "sortedIndices"
sortedIndicesFileName = os.path.join(savePath, sortedIndicesFileName + ".csv")

qAscendingOrderData = [[[] for i in range(3)] for j in range(totalJoints)] # [qValue, rowIndex, score]
qDotAscendingOrderData = [[] for i in range(totalJoints)] # [[qDotValues]]


if equaliser == "select":
	if not rarityFactor and not priorityFactor:
		print("Atleast one of rarityFactor and priorityFactor factors must be true for this subsampling method")
		sortedRows = []
	else:
		# sorting according to joint angle values
		for jointIndex in range(totalJoints):

			qData = rawData.iloc[:,1+jointIndex].tolist()
			qAscendingOrderData[jointIndex][0] = np.sort(np.array(qData)).tolist()
			qAscendingOrderData[jointIndex][1] = np.argsort(np.array(qData)).tolist()

			qDotAscendingOrderData[jointIndex] = rawData.iloc[qAscendingOrderData[jointIndex][1],1+totalJoints+jointIndex].tolist()
			
		#scoring
		for jointIndex in range(totalJoints):
			jointData = qAscendingOrderData[jointIndex][0]
			jointVelData = qDotAscendingOrderData[jointIndex]
			indexEnd = len(jointData)

			lastAboveIndex = 0
			lastBelowIndex = 0

			for index in range(indexEnd):  
				print(jointIndex, " ", index)    
				
				aboveIndex = lastAboveIndex
				belowIndex = lastBelowIndex

				indexScore = 0

				while not (jointData[index] - jointData[aboveIndex]  <= qBelt):
					aboveIndex = aboveIndex + 1
						
				while belowIndex < indexEnd and (jointData[belowIndex] - jointData[index] <= qBelt):
					belowIndex = belowIndex + 1

				for tempIndex in range(aboveIndex, belowIndex):
					if abs(jointVelData[tempIndex]-jointVelData[index]) < qDotBelt:
						indexScore = indexScore + 1

				lastAboveIndex = aboveIndex
				lastBelowIndex = belowIndex
				qAscendingOrderData[jointIndex][2] = qAscendingOrderData[jointIndex][2] + [indexScore]

		cumulativeScores = np.zeros((len(qAscendingOrderData[0][2]),totalJoints))

		for jointIndex in range(totalJoints):

			posIndices = np.array(qAscendingOrderData[jointIndex][1])
			velIndices = np.array(qDotAscendingOrderData[jointIndex])
			scores = np.array(qAscendingOrderData[jointIndex][2])
			organisedIndices = posIndices.argsort()
			organisedScores = scores[organisedIndices]
			cumulativeScores[:,jointIndex] = organisedScores

		outlierRows = []
		combinedWeight = np.zeros((np.size(cumulativeScores,0)))
		totalPoints = np.size(cumulativeScores,0)
		for rowIndex in reversed(range(np.size(cumulativeScores,0))):
			for jointIndex in range(totalJoints):
				deleteRow = False
				scoreIndex = cumulativeScores[rowIndex,jointIndex]
				qMax = qLimUpper[jointIndex]
				qMin = qLimLower[jointIndex]

				qDotMax = qDotLimUpper[jointIndex]
				qDotMin = qDotLimLower[jointIndex]

				if (scoreIndex < 10 and not (qMin <= rawData.iloc[rowIndex,1+jointIndex] <= qMax)) or (scoreIndex < 10 and not (qDotMin <= rawData.iloc[rowIndex,1+totalJoints+jointIndex] <= qDotMax)):
					if len(outlierRows) == 0 or not outlierRows[-1] == rowIndex: 
						outlierRows = outlierRows + [rowIndex]

				velSigmoidWeight = 1/(1 + math.exp((5/qDotMax)*abs(rawData.iloc[rowIndex,1+totalJoints+jointIndex])))

				if rarityFactor and priorityFactor:
					combinedWeight[rowIndex] = combinedWeight[rowIndex] + (1-scoreIndex/totalPoints)*(10*velSigmoidWeight)
				elif priorityFactor and not rarityFactor:
					combinedWeight[rowIndex] = combinedWeight[rowIndex] + velSigmoidWeight
				else:
					combinedWeight[rowIndex] = combinedWeight[rowIndex] + (1-scoreIndex/totalPoints)

		# drop outliers
		goodIndices = list(range(np.size(combinedWeight,0)))
		goodIndices = np.array(goodIndices).reshape((np.size(combinedWeight,0),1))

		for outlierIndex in outlierRows:
			goodIndices = np.delete(goodIndices, outlierIndex, axis=0)
			combinedWeight = np.delete(combinedWeight, outlierIndex, axis=0)

		inds = (-combinedWeight).argsort()
		sortedRows = goodIndices[inds]

else:
	sortedRows = list(range(len(rawData.index)))
	if equaliser == "random":
		random.shuffle(sortedRows)

print("Subsampling completed")

with open(sortedIndicesFileName, "w") as sortedIndicesFile:
	writer = csv.writer(sortedIndicesFile, lineterminator='\n')
	for val in sortedRows:
		writer.writerow([val])