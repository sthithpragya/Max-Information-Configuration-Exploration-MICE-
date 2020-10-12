#!/usr/bin/env python
import numpy as np
import csv
import os.path
import pandas
import math
import numpy as np
import yaml
import io
import random
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


# with open(r'/home/sthithpragya/Study/Thesis_test/catkin_ws/src/iiwa_ros/iiwa_data_collection/config/param.yaml') as stream:
with open(r'./../config/param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

totalJoints = paramLoaded["totalJoints"]
savePath = paramLoaded["savePath"]

AJ = "recordedActualJointData"
AT = "computedActualTaskData"
DJ = "recordedDesiredJointData"
DT = "computedDesiredTaskData"
FT = "feedTorqueData"
FB = "fbTorqueData"
JT = "jointTorqueData"
time = "recordedTimeData"

AJ = os.path.join(savePath, AJ + ".csv")
AT = os.path.join(savePath, AT + ".csv")
DJ = os.path.join(savePath, DJ + ".csv")
DT = os.path.join(savePath, DT + ".csv")
FT = os.path.join(savePath, FT + ".csv")
FB = os.path.join(savePath, FB + ".csv")
JT = os.path.join(savePath, JT + ".csv")
time = os.path.join(savePath, time + ".csv")


AJdata = pandas.read_csv(AJ, header=None, skiprows=1)
DJdata = pandas.read_csv(DJ, header=None, skiprows=1)
ATdata = pandas.read_csv(AT, header=None, skiprows=1)
DTdata = pandas.read_csv(DT, header=None, skiprows=1) 
FTdata = pandas.read_csv(FT, header=None, skiprows=1) 
FBdata = pandas.read_csv(FB, header=None, skiprows=1) 
JTdata = pandas.read_csv(JT, header=None, skiprows=1) 
timeData = pandas.read_csv(time) # list of time floats

# AJdata = AJdata.drop(0, axis=0) 
# DJdata = DJdata.drop(0, axis=0) 
# ATdata = ATdata.drop(0, axis=0) 
# DTdata = DTdata.drop(0, axis=0) 
# FTdata = FTdata.drop(0, axis=0) 
# FBdata = FBdata.drop(0, axis=0) 
# JTdata = JTdata.drop(0, axis=0) 
# timeData = timeData.drop(0, axis=0)

AJdata.index = range(len(AJdata))
DJdata.index = range(len(DJdata))
ATdata.index = range(len(ATdata))
DTdata.index = range(len(DTdata))
FTdata.index = range(len(FTdata))
FBdata.index = range(len(FBdata))
JTdata.index = range(len(JTdata))
timeData.index = range(len(timeData))




#--------------------------------------------------------------

infoFile = os.path.join(savePath, "info" + ".txt") 
info = open(infoFile, "w")

qMSE = [0 for i in range(totalJoints)]
qDotMSE = [0 for i in range(totalJoints)]

qMAE = [0 for i in range(totalJoints)]
qDotMAE = [0 for i in range(totalJoints)]

for jointIndex in range(totalJoints):

    qMSE[jointIndex] = (((AJdata.iloc[:,jointIndex] - DJdata.iloc[:,jointIndex]).pow(2)).sum(axis=0))/len(AJdata)
    qDotMSE[jointIndex] = (((AJdata.iloc[:,totalJoints+jointIndex] - DJdata.iloc[:,totalJoints+jointIndex]).pow(2)).sum(axis=0))/len(AJdata)

    qMAE[jointIndex] = (((AJdata.iloc[:,jointIndex] - DJdata.iloc[:,jointIndex]).abs()).sum(axis=0))/len(AJdata)
    qDotMAE[jointIndex] = (((AJdata.iloc[:,totalJoints+jointIndex] - DJdata.iloc[:,totalJoints+jointIndex]).abs()).sum(axis=0))/len(AJdata)

info.write("Joint angle mean sq. error:    " + str(qMSE) + "\n")
info.write("Joint velocity mean sq. error: " + str(qDotMSE) + "\n")

info.write("Joint angle mean abs error:    " + str(qMAE) + "\n")
info.write("Joint velocity mean abs error: " + str(qDotMAE) + "\n")


######## Visualisation

# plt.figure(1)
# ax = plt.axes(projection='3d')
# ax.plot3D(ATdata.iloc[:,0].tolist(), ATdata.iloc[:,1].tolist(), ATdata.iloc[:,2].tolist(), label='actual', color='red')
# ax.plot3D(DTdata.iloc[:,0].tolist(), DTdata.iloc[:,1].tolist(), DTdata.iloc[:,2].tolist(), label='desired', color='blue')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()

for jointIndex in range(totalJoints):
    plt.figure(20+jointIndex)
    plt.plot(timeData.iloc[:,0], AJdata.iloc[:,jointIndex], label='Actual')
    plt.plot(timeData.iloc[:,0], DJdata.iloc[:,jointIndex], label='Desired')
    plt.xlabel('time')
    plt.ylabel('joint angle')
    plt.legend()
    fileName = os.path.join(savePath, str(20+jointIndex) + '.png')
    plt.savefig(fileName, bbox_inches='tight')

# for jointIndex in range(totalJoints):
#     plt.figure(30+jointIndex)
#     plt.plot(AJdata.iloc[:,jointIndex].tolist(), AJdata.iloc[:,totalJoints+jointIndex].tolist(), label='Actual')
#     plt.plot(DJdata.iloc[:,jointIndex].tolist(), DJdata.iloc[:,totalJoints+jointIndex].tolist(), label='Desired')
#     plt.xlabel('q')
#     plt.ylabel('qDot')
#     plt.legend()
#     fileName = os.path.join(savePath, str(30+jointIndex) + '.png')
#     plt.savefig(fileName, bbox_inches='tight')

# for jointIndex in range(totalJoints):
#     plt.figure(40+jointIndex)
#     plt.plot(timeData.iloc[:,0].tolist(), FTdata.iloc[:,jointIndex].tolist(), label='FeedBack')
#     plt.plot(timeData.iloc[:,0].tolist(), FTdata.iloc[:,totalJoints+jointIndex].tolist(), label='FeedForward')
#     plt.plot(timeData.iloc[:,0].tolist(), FTdata.iloc[:,2*totalJoints+jointIndex].tolist(), label='Grav comp')
#     plt.plot(timeData.iloc[:,0].tolist(), JTdata.iloc[:,jointIndex].tolist(), label='TopicTauData')
#     plt.xlabel('time')
#     plt.ylabel('Joint torque')
#     plt.legend()
#     fileName = os.path.join(savePath, str(40+jointIndex) + '.png')
#     plt.savefig(fileName, bbox_inches='tight')

for jointIndex in range(totalJoints):
    plt.figure(50+jointIndex)
    plt.plot(timeData.iloc[:,0].tolist(), AJdata.iloc[:,totalJoints+jointIndex].tolist(), label='qDot actual')
    plt.plot(timeData.iloc[:,0].tolist(), DJdata.iloc[:,totalJoints+jointIndex].tolist(), label='qDot desired')
    plt.xlabel('time')
    plt.ylabel('joint velocity')
    plt.legend()
    fileName = os.path.join(savePath, str(50+jointIndex) + '.png')
    plt.savefig(fileName, bbox_inches='tight')

for jointIndex in range(totalJoints):
    plt.figure(60+jointIndex)
    plt.plot(timeData.iloc[:,0].tolist(), FBdata.iloc[:,jointIndex].tolist(), label='prop. feedback torque')
    plt.xlabel('time')
    plt.ylabel('torque')
    plt.legend()
    fileName = os.path.join(savePath, str(70+jointIndex) + '.png')
    plt.savefig(fileName, bbox_inches='tight')

for jointIndex in range(totalJoints):
    plt.figure(70+jointIndex)
    plt.plot(timeData.iloc[:,0].tolist(), FBdata.iloc[:,totalJoints+jointIndex].tolist(), label='der. feedback torque')
    plt.xlabel('time')
    plt.ylabel('torque')
    plt.legend()
    fileName = os.path.join(savePath, str(60+jointIndex) + '.png')
    plt.savefig(fileName, bbox_inches='tight')


for jointIndex in range(totalJoints):
    plt.figure(80+jointIndex)
    plt.plot(DJdata.iloc[:,jointIndex].tolist(), DJdata.iloc[:,totalJoints+jointIndex].tolist(), label='desired')
    plt.plot(AJdata.iloc[:,jointIndex].tolist(), AJdata.iloc[:,totalJoints+jointIndex].tolist(), label='actual')
    plt.xlabel('q')
    plt.ylabel('joint velocity')
    plt.legend()
    fileName = os.path.join(savePath, str(50+jointIndex) + '.png')
    plt.savefig(fileName, bbox_inches='tight')


for jointIndex in range(totalJoints):
    plt.figure(100+jointIndex)
    plt.plot(timeData.iloc[:,0].tolist(), FTdata.iloc[:,2*totalJoints+jointIndex].tolist(), label='int. feedback torque')
    plt.xlabel('time')
    plt.ylabel('torque')
    plt.legend()
    fileName = os.path.join(savePath, str(70+jointIndex) + '.png')
    plt.savefig(fileName, bbox_inches='tight')

# plt.show()