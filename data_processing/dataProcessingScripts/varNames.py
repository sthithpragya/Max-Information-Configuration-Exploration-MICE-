import yaml
import io
import math
import numpy as np

with open(r'param.yaml') as stream:
	paramLoaded = yaml.safe_load(stream)


savePath = paramLoaded["savePath"]

equaliser = paramLoaded["equaliser"]
priorityFactor = paramLoaded["priorityFactor"]
rarityFactor = paramLoaded["rarityFactor"]
bestPointCount = paramLoaded["bestPointCount"]
randomPointCount = paramLoaded["randomPointCount"]
recordingResultantTorque = paramLoaded["recordingResultantTorque"]
eigSpreadBatchSize = paramLoaded["eigSpreadBatchSize"]

useArtificialVel = paramLoaded["useArtVel"]
totalJoints = paramLoaded["totalJoints"]

qLimLower = paramLoaded["qLimLower"]
qLimUpper = paramLoaded["qLimUpper"]

qDotLimLower = paramLoaded["qDotLimLower"]
qDotLimUpper = paramLoaded["qDotLimUpper"]

qBound = paramLoaded["qBound"]
qDotBound = paramLoaded["qDotBound"]
qMargin = paramLoaded["qMargin"]
qDotMargin = paramLoaded["qDotMargin"]

qBelt = paramLoaded["qBelt"]
qDotBelt = paramLoaded["qDotBelt"]

toFilterQ = paramLoaded["toFilterQ"] 
toFilterQDot = paramLoaded["toFilterQDot"] 
toFilterTau = paramLoaded["toFilterTau"]

qFilterVal = paramLoaded["qFilterVal"] 
qDotFilterVal = paramLoaded["qDotFilterVal"] 
tauFilterVal = paramLoaded["tauFilterVal"] 
dt = paramLoaded["dt"]

jointAngleNames = [[] for i in range(totalJoints)]
jointVelNames = [[] for i in range(totalJoints)]
jointTorqueNames = [[] for i in range(totalJoints)]
predTorqueNames = [[] for i in range(totalJoints)]
jointAccNames = [[] for i in range(totalJoints)]
taskColNames = ['X', 'Y', 'Z']
timeName = ['time']

for jointIndex in range(totalJoints):
	jointAngleNames[jointIndex] = 'J' + str(jointIndex+1)
	jointVelNames[jointIndex] = 'J'+ str(jointIndex+1) + 'dot'
	jointAccNames[jointIndex] = 'J'+ str(jointIndex+1) + 'ddot'
	jointTorqueNames[jointIndex] = 'Tau'+ str(jointIndex+1)
	predTorqueNames[jointIndex] = "pred" + jointTorqueNames[jointIndex]

qResolutionList = []
qDotResolutionList = []

for jointIndex in range(totalJoints):
	qResolutionList.append(math.floor((qLimUpper[jointIndex]-qLimLower[jointIndex]-2*qMargin)/(2*qBound)))
	qDotResolutionList.append(math.floor((qDotLimUpper[jointIndex]-qDotLimLower[jointIndex]-2*qDotMargin)/(2*qDotBound)))



#############################################
'''
			 3|12 13 14 15
Index_2  Y   2|8  9  10 11
			 1|4  5  6  7
		     0|0  1  2  3
			  |___________
Index_1  X     0  1  2  3

Hypercube index = 4*Index_2 + Index_1
'''
def hyperCubeIndexHelper(index1, index2, index1RangeSize): # Index 1 along X, Index 2 along Y
	return index1RangeSize*index2 + index1 

def getHyperCubeIndex(indices, indicesRange): # indices -> 1 x totalJoints; indicesRange -> [ [Joint 1 index range {0,1}], [Joint 2 index range] ..... ]
	hyperCubeIndex = indices[0]
	indexRangeSize = len(indicesRange[0])
	for jointIndex in range(1,len(indicesRange)):
		hyperCubeIndex = hyperCubeIndexHelper(hyperCubeIndex, indices[jointIndex], indexRangeSize)
		indexRangeSize = indexRangeSize*len(indicesRange[jointIndex])

	return hyperCubeIndex
##############################################


