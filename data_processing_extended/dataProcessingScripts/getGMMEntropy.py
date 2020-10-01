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

componentScoresFileName = "GMMcomponentScores"
componentScoresFileName = os.path.join(savePath, componentScoresFileName + ".csv")

optimalComponentsFileName = "GMMOptimalComponentCount"
optimalComponentsFileName = os.path.join(savePath, optimalComponentsFileName + ".csv")

rawData = np.genfromtxt(rawDataFileName, delimiter=',', dtype=float, skip_header=1)

dataSize = np.shape(rawData)[0]
jointData = rawData[:min(dataSize, 183000),1:1+(2*totalJoints)]

dataSize =  np.shape(jointData)[0]
iterCount = int(dataSize/entropyBatchSize)

print("Data size: ", np.shape(jointData))

print("Getting the optimnal GMM")

##################

scores = [[] for i in range(len(componentCount))] # Component count vs log-likelihood scores
scoreOptimal = -np.inf
GMMgood = None
optimalComponents = 0
entropyMax = 0

for componentIndex in range(len(componentCount)): # Checking over different possible components

	print("componentIndex: ", componentIndex+1, "/", len(componentCount))
	print("Gaussians in consideration: ", componentCount[componentIndex])

	GMMtemp = mixture.GaussianMixture(n_components=componentCount[componentIndex], covariance_type='full', verbose=2, max_iter=1000, n_init=5, warm_start=True).fit(jointData)
	
	if not GMMtemp.converged_:
		scoreTemp = -np.inf
	else:
		scoreTemp = GMMtemp.score(jointData)
		
	scores[componentIndex] = [scoreTemp]

	# Overall entropy of the entire data
	logProbsTemp = GMMtemp.score_samples(jointData)
	logProbsTemp = np.clip(logProbsTemp,-1e10,np.max(logProbsTemp)) # Clipping to prevent numerical errors
	probsTemp = np.exp(logProbsTemp)
	entropyTemp = -np.dot(np.log(probsTemp),probsTemp)

	# Batchwise entropy for the GMM
	GMMEntropyTemp = [[] for i in range(iterCount)]
	for iter in range(iterCount):
		tempData = jointData[0:(iter+1)*entropyBatchSize,:]
		logProbsTempTemp = GMMtemp.score_samples(tempData)
		logProbsTempTemp = np.clip(logProbsTempTemp,-1e10,np.max(logProbsTempTemp)) # Clipping to prevent numerical errors
		probsTempTemp = np.exp(logProbsTempTemp)
		GMMEntropyTemp[iter] = [-np.dot(np.log(probsTempTemp),probsTempTemp)]

	# Checking for decrease in entropy somewhere
	entropiesTemp = np.array(GMMEntropyTemp)
	diffEntropies = np.diff(entropiesTemp)
	entropyIncrease = diffEntropies >= 0
	entropyIncrease = entropyIncrease.all()

	if entropyIncrease:
		if entropyTemp > entropyMax:
			entropyMax = entropyTemp
			GMMgood = GMMtemp
			scoreOptimal = scoreTemp
			optimalComponents = componentCount[componentIndex]

	print("scoreTemp:		", scoreTemp)
	print("entropyIncrease:	", entropyIncrease)
	print("entropyTemp:		", entropyTemp)
	
print("Overall scores:		", scores)
print("Optimnal score:		", scoreOptimal)
print("Optimal Components:	", optimalComponents)

# Saving the tuned GMM
GMMModelFileName = 'GMMtunedModel'
GMMModelFileName = os.path.join(savePath, GMMModelFileName)

joblib.dump(GMMgood, GMMModelFileName + '.joblib')

np.save(GMMModelFileName + '_weights', GMMgood.weights_, allow_pickle=False)
np.save(GMMModelFileName + '_means', GMMgood.means_, allow_pickle=False)
np.save(GMMModelFileName + '_covariances', GMMgood.covariances_, allow_pickle=False)

# Computing the batchwise entropies
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

file2 = open(componentScoresFileName, 'w+', newline ='')   
with file2:     
	write = csv.writer(file2)
	write.writerows(scores)

file3 = open(optimalComponentsFileName, 'w+', newline ='')   
with file3:     
	write = csv.writer(file3)
	write.writerows([[optimalComponents]])