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

with open(r'param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

dataCollectionParamLoc = paramLoaded["dataCollectionParamLoc"]
with open(dataCollectionParamLoc,"r") as dataCollectionStream:
    dataCollectionParam = yaml.safe_load(dataCollectionStream)

savePath = paramLoaded["savePath"]
equaliser = paramLoaded["equaliser"]

totalJoints = dataCollectionParam["totalJoints"]
qLimList = dataCollectionParam["qLimList"]
qDotLimList = dataCollectionParam["qDotLimList"]
qBound = dataCollectionParam["qBound"]
qDotBound = dataCollectionParam["qDotBound"]
margin = dataCollectionParam["margin"]

qBelt = paramLoaded["qBelt"]
qDotBelt = paramLoaded["qDotBelt"]
qResolutionList = []
qDotResolutionList = []

for jointIndex in range(totalJoints):
    qResolutionList.append(math.floor(int((qLimList[jointIndex]-margin)/qBound)))
    qDotResolutionList.append(math.floor(int(qDotLimList[jointIndex]/qDotBound)))

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

        qMax = qLimList[jointIndex]
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
            qMax = qLimList[jointIndex]
            qDotMax = qDotLimList[jointIndex]

            if (scoreIndex < 10 and abs(rawData.iloc[rowIndex,1+jointIndex]) > qMax) or (scoreIndex < 10 and abs(rawData.iloc[rowIndex,1+totalJoints+jointIndex]) > qDotMax):
                if len(outlierRows) == 0 or not outlierRows[-1] == rowIndex: 
                    outlierRows = outlierRows + [rowIndex]

            # posSigmoidWeight = 1/(1 + math.exp((5/qMax)*abs(rawData.iloc[rowIndex,1+jointIndex])))
            velSigmoidWeight = 1/(1 + math.exp((5/qDotMax)*abs(rawData.iloc[rowIndex,1+totalJoints+jointIndex])))

            # radialDist = math.pow((5/qDotMax)*abs(rawData.iloc[rowIndex,1+totalJoints+jointIndex]),2) + math.pow((5/qMax)*abs(rawData.iloc[rowIndex,1+jointIndex]),2)        
            # uniSigmoidWeight = 1/(1 + math.exp(math.sqrt(radialDist)))

            combinedWeight[rowIndex] = combinedWeight[rowIndex] + (1-scoreIndex/totalPoints)*(10*velSigmoidWeight)
            
            # combinedWeight[rowIndex] = combinedWeight[rowIndex] + (1-scoreIndex/totalPoints)*(10*posSigmoidWeight + 10*velSigmoidWeight)
            # combinedWeight[rowIndex] = combinedWeight[rowIndex] + (1-scoreIndex/totalPoints)*uniSigmoidWeight


    # drop outliers from the weights
    goodIndices = list(range(np.size(combinedWeight,0)))
    goodIndices = np.array(goodIndices).reshape((np.size(combinedWeight,0),1))

    for outlierIndex in outlierRows:
        goodIndices = np.delete(goodIndices, outlierIndex, axis=0)
        combinedWeight = np.delete(combinedWeight, outlierIndex, axis=0)

    inds = (-combinedWeight).argsort()
    sortedRows = goodIndices[inds]

else:
    sortedRows = list(range(len(rawData.index)))
    print(sortedRows)

with open(sortedIndicesFileName, "w") as sortedIndicesFile:
    writer = csv.writer(sortedIndicesFile, lineterminator='\n')
    for val in sortedRows:
        writer.writerow([val])