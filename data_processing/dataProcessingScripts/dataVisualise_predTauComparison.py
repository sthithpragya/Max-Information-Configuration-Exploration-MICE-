#!/usr/bin/env python
import numpy as np
import csv
import os.path
import pandas
import math
import numpy as np
import yaml
import io
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from numpy import genfromtxt

# from dataProcess import PAD
# from dataProcess import PAD
# from dataProcess import savePath
# from dataProcess import timeName, jointAngleNames, jointVelNames, jointAccNames, jointTorqueNames, jointGravityTorqueNames, taskColNames
#   from dataProcess import qResolutionList, qDotResolutionList


with open(r'param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

dataCollectionParamLoc = paramLoaded["dataCollectionParamLoc"]
with open(dataCollectionParamLoc,"r") as dataCollectionStream:
    dataCollectionParam = yaml.safe_load(dataCollectionStream)


savePath = paramLoaded["savePath"]
jointAngleNames = paramLoaded["jointAngleNames"]
jointVelNames =  paramLoaded["jointVelNames"]
jointAccNames =  paramLoaded["jointAccNames"]
jointTorqueNames =  paramLoaded["jointTorqueNames"]
jointGravityTorqueNames =  paramLoaded["jointGravityTorqueNames"]
taskColNames =  paramLoaded["taskColNames"]
timeName =  paramLoaded["timeName"]

jointAngleNames = paramLoaded["jointAngleNames"]
jointVelNames =  paramLoaded["jointVelNames"]
jointAccNames =  paramLoaded["jointAccNames"]
jointTorqueNames =  paramLoaded["jointTorqueNames"]

filteredTauFileName = "filteredTauData"
groupedFilteredTauFileName = "groupedFilteredTauData"
filteredJointDataFileName = "filteredSubSampledData"
initPredTauFileName = "initParametricPredTau"
optimPredTauFileName_abs = "optimParametricPredTau_absolute"
optimPredTauFileName_25 = "optimParametricPredTau_25%"
optimPredTauFileName_50 = "optimParametricPredTau_50%"

optimPredTauFileName_abs_g = "optimParametricPredTau_absolute_grouped"
optimPredTauFileName_25_g = "optimParametricPredTau_25%_grouped"
optimPredTauFileName_50_g = "optimParametricPredTau_50%_grouped"

# optimPredTauFileName_reg = "optimParametricPredTau_lineRegression"

filteredTauFileName = os.path.join(savePath, filteredTauFileName + ".csv")
groupedFilteredTauFileName = os.path.join(savePath, groupedFilteredTauFileName + ".csv")
filteredJointDataFileName = os.path.join(savePath, filteredJointDataFileName + ".csv")
initPredTauFileName = os.path.join(savePath, initPredTauFileName + ".csv")

optimPredTauFileName_abs = os.path.join(savePath, optimPredTauFileName_abs + ".csv")
optimPredTauFileName_25 = os.path.join(savePath, optimPredTauFileName_25 + ".csv")
optimPredTauFileName_50 = os.path.join(savePath, optimPredTauFileName_50 + ".csv")

optimPredTauFileName_abs_g = os.path.join(savePath, optimPredTauFileName_abs_g + ".csv")
optimPredTauFileName_25_g = os.path.join(savePath, optimPredTauFileName_25_g + ".csv")
optimPredTauFileName_50_g = os.path.join(savePath, optimPredTauFileName_50_g + ".csv")
# optimPredTauFileName_reg = os.path.join(savePath, optimPredTauFileName_reg + ".csv")


filteredTau = genfromtxt(filteredTauFileName, delimiter=',')
filteredTau_g = genfromtxt(groupedFilteredTauFileName, delimiter=',')
filteredJointData = genfromtxt(filteredJointDataFileName, delimiter=',')
tau_init = genfromtxt(initPredTauFileName, delimiter=',')

tau_abs = genfromtxt(optimPredTauFileName_abs, delimiter=',')
tau_25 = genfromtxt(optimPredTauFileName_25, delimiter=',')
tau_50 = genfromtxt(optimPredTauFileName_50, delimiter=',')

tau_abs_g = genfromtxt(optimPredTauFileName_abs_g, delimiter=',')
tau_25_g = genfromtxt(optimPredTauFileName_25_g, delimiter=',')
tau_50_g = genfromtxt(optimPredTauFileName_50_g, delimiter=',')
# tau_reg = genfromtxt(optimPredTauFileName_reg, delimiter=',')


PAD = "selectEqualisedData"
PDD = "processedDesiredData"
PAD = os.path.join(savePath, PAD + ".csv")
PDD = os.path.join(savePath, PDD + ".csv")
vizActualData = pandas.read_csv(PAD, header=0)
vizDesiredData = pandas.read_csv(PDD, header=0)



totalJoints = dataCollectionParam["totalJoints"]
qLimList = dataCollectionParam["qLimList"]
qDotLimList = dataCollectionParam["qDotLimList"]
qBound = dataCollectionParam["qBound"]
qDotBound = dataCollectionParam["qDotBound"]
margin = dataCollectionParam["margin"]

qResolutionList = []
qDotResolutionList = []

for jointIndex in range(totalJoints):
    qResolutionList.append(math.floor((qLimList[jointIndex]-margin)/qBound))
    qDotResolutionList.append(math.floor(qDotLimList[jointIndex]/qDotBound))

grid = [[[] for i in range(2)] for jointIndex in range(totalJoints)]

for jointIndex in range(totalJoints):

    qResolution = int(qResolutionList[jointIndex])
    qDotResolution = int(qDotResolutionList[jointIndex])
    qSpace = np.linspace(-qLimList[jointIndex]+margin,
                            qLimList[jointIndex]-margin, num=qResolution+1)

    qDotSpace = np.linspace(-qDotLimList[jointIndex],
                            qDotLimList[jointIndex], num=qDotResolution+1)

    grid[jointIndex][0] = qSpace.tolist()
    grid[jointIndex][1] = qDotSpace.tolist()


plt.figure(11)    
plt.plot(vizActualData.time.tolist(), tau_init[:,0].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,0].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,0].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,0].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,0].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau1.tolist(), label='J1_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,0].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J1ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau1')
plt.legend()

plt.figure(12)    
plt.plot(vizActualData.time.tolist(), tau_init[:,1].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,1].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,1].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,1].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,1].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau2.tolist(), label='J2_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,1].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J2ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+1], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau2')
plt.legend()

plt.figure(13)    
plt.plot(vizActualData.time.tolist(), tau_init[:,2].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,2].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,2].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,2].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,2].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau3.tolist(), label='J3_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,2].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J3ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+2], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau3')
plt.legend()

plt.figure(14)    
plt.plot(vizActualData.time.tolist(), tau_init[:,3].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,3].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,3].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,3].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,3].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau4.tolist(), label='J4_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,3].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J4ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+3], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau4')
plt.legend()

plt.figure(15)    
plt.plot(vizActualData.time.tolist(), tau_init[:,4].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,4].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,4].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,4].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,4].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau5.tolist(), label='J5_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,4].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J5ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+4], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau5')
plt.legend()

plt.figure(16)    
plt.plot(vizActualData.time.tolist(), tau_init[:,5].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,5].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,5].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,5].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,5].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau6.tolist(), label='J6_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,5].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J6ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+5], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau6')
plt.legend()

plt.figure(17)    
plt.plot(vizActualData.time.tolist(), tau_init[:,6].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs[:,6].tolist(), label='tau_abs')
plt.plot(vizActualData.time.tolist(), tau_25[:,6].tolist(), label='tau_25')
plt.plot(vizActualData.time.tolist(), tau_50[:,6].tolist(), label='tau_50')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,6].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau7.tolist(), label='J7_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau[:,6].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J7ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+6], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau7')
plt.legend()

#---------------------- grouped
plt.figure(21)    
plt.plot(vizActualData.time.tolist(), tau_init[:,0].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,0].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,0].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,0].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,0].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau1.tolist(), label='J1_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,0].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J1ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau1')
plt.legend()

plt.figure(22)    
plt.plot(vizActualData.time.tolist(), tau_init[:,1].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,1].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,1].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,1].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,1].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau2.tolist(), label='J2_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,1].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J2ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+1], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau2')
plt.legend()

plt.figure(23)    
plt.plot(vizActualData.time.tolist(), tau_init[:,2].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,2].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,2].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,2].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,2].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau3.tolist(), label='J3_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,2].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J3ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+2], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau3')
plt.legend()

plt.figure(24)    
plt.plot(vizActualData.time.tolist(), tau_init[:,3].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,3].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,3].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,3].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,3].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau4.tolist(), label='J4_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,3].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J4ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+3], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau4')
plt.legend()

plt.figure(25)    
plt.plot(vizActualData.time.tolist(), tau_init[:,4].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,4].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,4].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,4].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,4].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau5.tolist(), label='J5_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,4].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J5ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+4], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau5')
plt.legend()

plt.figure(26)    
plt.plot(vizActualData.time.tolist(), tau_init[:,5].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,5].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,5].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,5].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,5].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau6.tolist(), label='J6_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,5].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J6ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+5], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau6')
plt.legend()

plt.figure(27)    
plt.plot(vizActualData.time.tolist(), tau_init[:,6].tolist(), label='tau_init')
plt.plot(vizActualData.time.tolist(), tau_abs_g[:,6].tolist(), label='tau_abs_g')
plt.plot(vizActualData.time.tolist(), tau_25_g[:,6].tolist(), label='tau_25_g')
plt.plot(vizActualData.time.tolist(), tau_50_g[:,6].tolist(), label='tau_50_g')
# plt.plot(vizActualData.time.tolist(), tau_reg[:,6].tolist(), label='tau_reg')
plt.plot(vizActualData.time.tolist(), vizActualData.Tau7.tolist(), label='J7_tau')
# plt.plot(vizActualData.time.tolist(), filteredTau_g[:,6].tolist(), label='tau_filtered')
# plt.plot(vizActualData.time.tolist(), vizActualData.J7ddot.tolist(), label='acc')
# plt.plot(vizActualData.time.tolist(), filteredJointData[:,2*totalJoints+6], label='filtered acc')
plt.xlabel('time')
plt.ylabel('tau7')
plt.legend()




plt.show()