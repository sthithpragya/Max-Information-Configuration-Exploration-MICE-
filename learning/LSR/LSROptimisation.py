#!/usr/bin/env python
import csv
import os.path
import pandas
import math
import numpy as np
import yaml
import io
import json
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy import optimize
from sklearn.linear_model import LinearRegression
from joblib import dump, load
from sklearn.model_selection import train_test_split
from scipy.optimize import lsq_linear

with open('./../param.yaml',"r") as learningStream:
    paramLoaded = yaml.safe_load(learningStream)

savePath = paramLoaded["savePath"]
linkCount = paramLoaded["linkCount"]
linkParam = paramLoaded["linkParam"]
zeroParamCount = paramLoaded["zeroParamCount"]
tauDimension = paramLoaded["totalJoints"]
totalJoints = paramLoaded["totalJoints"]
bounded = paramLoaded["bounded"]

regressorYFileName = "regressorYData_train"
initPredTauFileName = "initParametricPredTau"
optimPredTauFileName = "optimParametricPredTau_" + bounded
filteredTauFileName = "trainTauData_filtered"

regressorYFileName = os.path.join(savePath, regressorYFileName + ".csv")
initPredTauFileName = os.path.join(savePath, initPredTauFileName + ".csv")
optimPredTauFileName = os.path.join(savePath, optimPredTauFileName + ".csv")
filteredTauFileName = os.path.join(savePath, filteredTauFileName + ".csv")

print("Reading Data")

regressorYData = pandas.read_csv(regressorYFileName, header=None)
regressorYData = regressorYData.iloc[:,zeroParamCount:]

tauData = pandas.read_csv(filteredTauFileName, header=None) # using the filtered tau data

unknownParamCount = linkCount*linkParam - zeroParamCount
dataEntryCount = int(len(regressorYData.index)/tauDimension)

print("dataEntryCount       : ", dataEntryCount)
print("unknownParamCount    : ", unknownParamCount)

RyData = regressorYData.to_numpy() #size 875000, 71
tData = tauData.to_numpy() # size 125000, 7

X = RyData
Y = np.squeeze(tData.reshape((-1,1)))
print("X.shape:", X.shape)
print("Y.shape:", Y.shape)

# SETTING THE BOUNDS
# Borrowed from original URDF
'''
>   Usual parameters per link -> [#m lx ly lz Ixx Ixy Ixz Iyy Iyz Izz] (10 in all)
>   Sequentially fill the link-specific physical parameters, except the parameters 
    corresponding to zeroParamCount.

>   For instance, here the URDF suggests a total of 10 link in the robot, with the 
    first 2 of them being 'fixed'. Consequently, they do not affect the dynamics. 
    Additionally, even for the first link to actually affect the dynamics, only its 
    Izz parameter contributes to the dynamics. This leads to a total of 10x2 + 9 = 29 
    parameters being non-contributing.

>   NOTE: The zeroParamCount functionality cannot be used to cater to non-contributing parameters 
    of any intermediate of terminal links.
'''
l1Param = [0.02] # Izz
l2InitParam = [4, 0.0003, 0.059, 0.042, 0.05,  0,  0, 0.018, 0, 0.044]  
l3InitParam = [3, 0, 0.03, 0.13, 0.08, 0,  0, 0.075, 0, 0.01]           
l4InitParam = [2.7, 0, 0.067, 0.034, 0.03,  0,  0, 0.01, 0, 0.029]          
l5InitParam = [1.7, 0.0001, 0.021, 0.076, 0.02,  0,  0, 0.018, 0, 0.005]                
l6InitParam = [1.8, 0, 0.0006, 0.0004, 0.005,  0,  0, 0.0036, 0, 0.0047]            
l7InitParam = [0.3, 0, 0, 0.02, 0.001,  0,  0, 0.001, 0, 0.001]         
l8InitParam = [1e-6, 0, 0, 0, 1,  0,  0, 1, 0, 1]           

L = [l2InitParam, l3InitParam, l4InitParam, l5InitParam, l6InitParam, l7InitParam, l8InitParam]
x0 = []
for i in range(7):
    # m mlx mly mlz Ixx Ixy ... Izz
    L[i][1] = L[i][1] * L[i][0] 
    L[i][2] = L[i][2] * L[i][0]
    L[i][3] = L[i][3] * L[i][0]
    x0 = x0 + L[i]
    
x0 = l1Param + x0 # intial point
X0 = np.array(x0)

# Saving the joint-torque predictions from non-optimised (original) parameters
initPredTau = regressorYData.to_numpy().dot(X0.reshape((X0.size,1)))
initPredTauReshape = initPredTau.reshape((int(initPredTau.size/tauDimension),tauDimension))
np.savetxt(initPredTauFileName, initPredTauReshape, delimiter=',')

# Bounds on the parameters to be found
dummy = 0.0001
zeroArray = np.full(X0.shape, 0)
filler = dummy*(X0 == zeroArray)
lowerBound = []
upperBound = []
    
if bounded == "absolute":
    massBoundL = 1e-7
    massBoundU = 8
    mCOMBoundL = 0
    mCOMBoundU = 1.6 # assuming max |l_com| = 0.2 m
    inertiaBoundL = [0.001, 0, 0, 0.001, 0, 0.001]
    inertiaBoundU = [1.2, 0.01, 0.01, 1.2, 0.01, 1.2]
    linkBoundL = [massBoundL] + [mCOMBoundL for i in range(3)] + inertiaBoundL 
    linkBoundU = [massBoundU] + [mCOMBoundU for i in range(3)] + inertiaBoundU 
    lowerBound = [inertiaBoundL[-1]] + linkBoundL*7
    upperBound = [inertiaBoundU[-1]] + linkBoundU*7

elif bounded == "25%":

    lowerBound = ((X0+filler)*0.75).tolist()
    upperBound = ((X0+filler)*1.25).tolist()

elif bounded == "50%":
    lowerBound = ((X0+filler)*0.5).tolist()
    upperBound = ((X0+filler)*1.5).tolist()

else:
    print("incorrect bounded selection")

boundGlobal = (np.array(lowerBound), np.array(upperBound))

print("training in progress")
res = lsq_linear(X, Y, bounds=boundGlobal, verbose=2, method='trf')

## Saving the model for possible future use
# modelName = 'lsqOptimParams_' + bounded + '.joblib'
# dump(res, modelName) 

print("training ended")
scoreFileName = "LSR_Report.txt"
scoreFile = open(scoreFileName, "a+")

scoreFile.writelines("\n" + "----------------------------------------------------------" + "\n")
scoreFile.writelines("using linear least squares")
print("data size: ", X.shape)
print(res)
scoreFile.write("Result using " + bounded + " bounds \n")
print(res, file=scoreFile)

# Saving the joint-torque predictions from optimised parameters
XOptim = res["x"]
XOptimReshape = XOptim.reshape((XOptim.size,1))
optimPredTau = regressorYData.to_numpy().dot(XOptimReshape)
optimPredTauReshape = optimPredTau.reshape((int(optimPredTau.size/tauDimension),tauDimension))
np.savetxt(optimPredTauFileName, optimPredTauReshape, delimiter=',')