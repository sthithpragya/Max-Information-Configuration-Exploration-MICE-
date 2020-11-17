import yaml
import io
import math
import numpy as np

with open(r'param.yaml') as stream:
	paramLoaded = yaml.safe_load(stream)


savePath = paramLoaded["savePath"]

trainSize = paramLoaded["trainSize"]
testSize = paramLoaded["testSize"]

recordingResultantTorque = paramLoaded["recordingResultantTorque"]

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

toFilterQ = paramLoaded["toFilterQ"] 
toFilterQDot = paramLoaded["toFilterQDot"] 
toFilterTau = paramLoaded["toFilterTau"]

qFilterVal = paramLoaded["qFilterVal"] 
qDotFilterVal = paramLoaded["qDotFilterVal"] 
tauFilterVal = paramLoaded["tauFilterVal"] 
dt = paramLoaded["dt"]

useForTesting = paramLoaded["useForTesting"]
testMode = paramLoaded["testMode"]

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

grid = [[[] for i in range(2)] for jointIndex in range(totalJoints)]

for jointIndex in range(totalJoints):

    qResolution = int(qResolutionList[jointIndex])
    qDotResolution = int(qDotResolutionList[jointIndex])
    qSpace = np.linspace(qLimLower[jointIndex]+qMargin,
						 qLimUpper[jointIndex]-qMargin, num=qResolution+1)
    
    qDotSpace = np.linspace(-qDotLimLower[jointIndex]+qDotMargin,
							qDotLimUpper[jointIndex]-qDotMargin, num=qDotResolution+1)

    grid[jointIndex][0] = qSpace.tolist()
    grid[jointIndex][1] = qDotSpace.tolist()
