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


import itertools
from sklearn import mixture


rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

GMMEntropyFileName = "GMMEntropy"
GMMEntropyFileName = os.path.join(savePath, GMMEntropyFileName + ".csv")

rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)
jointData = rawData[:,1:1+(2*totalJoints)]

dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/entropyBatchSize)

print("Calculating the cross-randomness")

# Calculating the variances
GMMEntropy = [[] for i in range(iterCount)]
# for iter in range(iterCount):
for iter in range(5):
	print("Iteration count: ", iter+1, "/", iterCount)

	tempData = jointData[0:(iter+1)*entropyBatchSize,:]

	tempMean = np.mean(tempData, axis=0)
	tempCovar = np.cov(np.transpose(tempData))

	tempGMMEntropy = 0

	GMMTemp = mixture.GaussianMixture(n_components=componentCount, covariance_type='full', verbose=2).fit(jointData)

	probDist = GMMTemp.predict_proba(jointData)
	GMMEntropy[iter] = np.dot(probDist,np.log(probDist))

# Writing the variances to csv
file = open(GMMEntropyFileName, 'w+', newline ='')   
with file:     
	write = csv.writer(file)
	write.writerows(GMMEntropy)