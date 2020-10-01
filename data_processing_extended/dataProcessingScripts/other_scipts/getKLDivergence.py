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
from scipy.stats import multivariate_normal
from varNames import *

rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

KLDFileName = "KLDivergence"
KLDFileName = os.path.join(savePath, KLDFileName + ".csv")

rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)
jointData = rawData[:,1:1+(2*totalJoints)]

dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/entropyBatchSize)

Qmean = (np.array(qLimUpper+qDotLimUpper) + np.array(qLimLower+qDotLimLower))/2
Qcovar = np.identity(2*totalJoints)

Q = multivariate_normal.pdf(jointData, mean=Qmean, cov=Qcovar)

print("Calculating the cross-randomness")
fileKLD = open(KLDFileName, 'w+', newline ='')   

# Calculating the variances
KLDivergence = [[] for i in range(iterCount)]
# for iter in range(iterCount):
for iter in range(30):
	print("Iteration count: ", iter+1, "/", iterCount)

	tempData = jointData[0:(iter+1)*entropyBatchSize,:]

	tempMean = np.mean(tempData, axis=0)
	tempCovar = np.cov(np.transpose(tempData))

	tempKLD = 0

	for tempIndex in range(np.shape(tempData)[0]):
		qData = tempData[tempIndex,:]
		P = multivariate_normal.pdf(qData, mean=tempMean, cov=tempCovar)
		Qtemp = Q[tempIndex]
		tempKLD = tempKLD + P*np.log(P/Qtemp)

	KLDivergence[iter] = [tempKLD]

# Writing the variances to csv
file = open(KLDFileName, 'w+', newline ='')   
with file:     
	write = csv.writer(file)
	write.writerows(KLDivergence)