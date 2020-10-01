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
import joblib
import pickle
from varNames import *


import itertools
from sklearn import mixture


rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")

GMMEntropyFileName = "GMMEntropy"
GMMEntropyFileName = os.path.join(savePath, GMMEntropyFileName + ".csv")


rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)
jointData = rawData[:,1:1+(2*totalJoints)]

print("Data size: ", np.shape(jointData))

print("Computing entropy for the optimal GMM")

GMMModelFileName = 'GMMtunedModel'
GMMModelFileName = os.path.join(savePath, GMMModelFileName)

GMMgood = joblib.load(GMMModelFileName+'.joblib')

# Computing the batchwise entropies
dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/entropyBatchSize)

GMMEntropy = [[] for i in range(iterCount)]

for iter in range(iterCount):
	tempData = jointData[0:(iter+1)*entropyBatchSize,:]
	
	logProbs = GMMgood.score_samples(tempData)
	logProbs = np.clip(logProbs,-1e10,np.max(logProbs)) # Clipping to prevent numerical errors
	probs = np.exp(logProbs)

	GMMEntropy[iter] = [-np.dot(np.log(probs),probs)]
	
############
# Writing data to csv
file1 = open(GMMEntropyFileName, 'w+', newline ='')   
with file1:     
	write = csv.writer(file1)
	write.writerows(GMMEntropy)