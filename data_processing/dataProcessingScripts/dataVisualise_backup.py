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

PAD = "processedActualData"
PDD = "processedDesiredData"
PAD = os.path.join(savePath, PAD + ".csv")
PDD = os.path.join(savePath, PDD + ".csv")
vizActualData = pandas.read_csv(PAD, header=0)
vizDesiredData = pandas.read_csv(PDD, header=0)





# EEQ = "elimEqualisedData"
# EEQ = os.path.join(savePath, EEQ + ".csv") # equalised data saved to learningDataDirectory
# EEQdata = pandas.read_csv(EEQ, names=timeName+jointAngleNames+jointVelNames+jointAccNames+jointTorqueNames+jointGravityTorqueNames+taskColNames)

SEQ = "trainData"
SEQ = os.path.join(savePath, SEQ + ".csv") # equalised data saved to learningDataDirectory
SEQdata = pandas.read_csv(SEQ, header=0)

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

# setting up the graph

# plt.figure(1)
# ax = plt.axes(projection='3d')
# ## Data for a three-dimensional line

# ax.plot3D(vizActualData.X.tolist(), vizActualData.Y.tolist(), vizActualData.Z.tolist(), label='actual', color='red')
# ax.plot3D(vizDesiredData.X.tolist(), vizDesiredData.Y.tolist(), vizDesiredData.Z.tolist(), label='desired', color='blue')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()

# plt.figure(2)
# plt.plot(vizDesiredData.time.tolist(), vizDesiredData.J1.tolist(), label='J1_desired')
# plt.plot(vizDesiredData.time.tolist(), vizDesiredData.J2.tolist(), label='J2_desired')
# plt.plot(vizActualData.time.tolist(), vizActualData.J1.tolist(), label='J1_actual')
# plt.plot(vizActualData.time.tolist(), vizActualData.J2.tolist(), label='J2_actual')
# plt.legend()
# jointAngleFile = os.path.join(savePath, 'jointAngleComparison' + '.png')
# # plt.savefig(jointAngleFile, bbox_inches='tight')

# plt.figure(3)
# plt.plot(vizActualData.time.tolist(), vizActualData.J1dot.tolist(), label='J1dot_actual')
# plt.plot(vizActualData.time.tolist(), vizActualData.J2dot.tolist(), label='J2dot_actual')
# plt.plot(vizActualData.time.tolist(), vizActualData.J3dot.tolist(), label='J3dot_actual')
# plt.legend()
# jointSpeedFile = os.path.join(savePath, 'actualJointSpeeds' + '.png')
# # plt.savefig(jointSpeedFile, bbox_inches='tight')

# plt.figure(4)
# plt.plot(vizActualData.time.tolist()[1:], vizActualData.J1ddot.tolist()[1:], label='J1ddot_actual')
# plt.plot(vizActualData.time.tolist()[1:], vizActualData.J2ddot.tolist()[1:], label='J2ddot_actual')
# plt.plot(vizActualData.time.tolist()[1:], vizActualData.J6ddot.tolist()[1:], label='J6ddot_actual')
# plt.legend()
# jointAccFile = os.path.join(savePath, 'actualJointAcc' + '.png')
# # plt.savefig(jointAccFile, bbox_inches='tight')


# plt.figure(5)
# plt.plot(vizActualData.J1.tolist()[1:], vizActualData.Tau3.tolist()[1:], label='J3Pos vs J3Torque')
# plt.legend()
# posVsTau = os.path.join(savePath, 'J3Pos_vs_J3Torque' + '.png')
# plt.savefig(posVsTau, bbox_inches='tight')

# plt.figure(51)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.time.tolist()[1:], vizActualData.J1.tolist()[1:], vizActualData.Tau3.tolist()[1:], label='time vs J3Pos vs J3Torque')
# ax.set_xlabel('time')
# ax.set_ylabel('qt')
# ax.set_zlabel('tau')
# ax.legend()

# # # phase plots (q, qdot, tau) - desired vs actual
# plt.figure(11)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), vizActualData.Tau1.tolist(), label='J1_actual')
# ax.plot3D(vizDesiredData.J1.tolist(), vizDesiredData.J1dot.tolist(), label='J1_desired')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()


#--------------------------------------
# plt.figure(12)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), vizActualData.Tau1.tolist(), label='Joint 1 Trajectory')
# # ax.plot3D(vizDesiredData.J2.tolist(), vizDesiredData.J2dot.tolist(), label='J2_desired')
# ax.set_xlabel('Joint 1 Angle (in radians)', fontsize = 18)
# ax.set_ylabel('Joint 1 Velocity (in radians/seconde)', fontsize = 18)
# ax.set_zlabel('Joint 1 Torque (in Newton metre)', fontsize = 18)
# ax.legend(fontsize=18)

# plt.figure(13)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J3.tolist(), vizActualData.J3dot.tolist(), vizActualData.Tau3.tolist(), label='J3_actual')
# ax.plot3D(vizDesiredData.J3.tolist(), vizDesiredData.J3dot.tolist(), label='J3_desired')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(14)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J4.tolist(), vizActualData.J4dot.tolist(), vizActualData.Tau4.tolist(), label='J4_actual')
# ax.plot3D(vizDesiredData.J4.tolist(), vizDesiredData.J4dot.tolist(), label='J4_desired')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(15)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J5.tolist(), vizActualData.J5dot.tolist(), vizActualData.Tau5.tolist(), label='J5_actual')
# ax.plot3D(vizDesiredData.J5.tolist(), vizDesiredData.J5dot.tolist(), label='J5_desired')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(16)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J6.tolist(), vizActualData.J6dot.tolist(), vizActualData.Tau6.tolist(), label='J6_actual')
# ax.plot3D(vizDesiredData.J6.tolist(), vizDesiredData.J6dot.tolist(), label='J6_desired')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(17)
# ax = plt.axes(projection='3d')
# ax.plot3D(vizActualData.J7.tolist(), vizActualData.J7dot.tolist(), vizActualData.Tau7.tolist(), label='J7_actual')
# ax.plot3D(vizDesiredData.J7.tolist(), vizDesiredData.J7dot.tolist(), label='J7_desired')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()


#----------------------------------------

# # des vs actual in 2d
# # # phase plots (q, qdot, tau) - desired vs actual
plt.figure(11)
localGrid = grid[0]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=100)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=100)
    
plt.plot(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), label='J1_actual',zorder=0)
plt.plot(vizDesiredData.J1.tolist(), vizDesiredData.J1dot.tolist(), label='J1_desired',zorder=50, color='darkorange', alpha=0.5)

plt.xlabel('q')
plt.ylabel('qDot')
plt.legend()

plt.figure(12)
localGrid = grid[1]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1],zorder=100)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1],zorder=100)

plt.plot(vizActualData.J2.tolist(), vizActualData.J2dot.tolist(), label='J2_actual',zorder=50, color='blue')
plt.plot(vizDesiredData.J2.tolist(), vizDesiredData.J2dot.tolist(), label='J2_desired',zorder=0, color='red')

plt.xlabel('q')
plt.ylabel('qDot')
plt.legend()

plt.figure(13)
localGrid = grid[2]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1],zorder=100)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1],zorder=100)
plt.plot(vizActualData.J3.tolist(), vizActualData.J3dot.tolist(), label='J3_actual')
plt.xlabel('q')
plt.ylabel('qDot')
plt.legend()

plt.figure(14)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J4.tolist(), vizActualData.J4dot.tolist(), label='J4_actual')
ax.plot3D(vizDesiredData.J4.tolist(), vizDesiredData.J4dot.tolist(), label='J4_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(15)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J5.tolist(), vizActualData.J5dot.tolist(), label='J5_actual')
ax.plot3D(vizDesiredData.J5.tolist(), vizDesiredData.J5dot.tolist(), label='J5_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(16)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J6.tolist(), vizActualData.J6dot.tolist(), label='J6_actual')
ax.plot3D(vizDesiredData.J6.tolist(), vizDesiredData.J6dot.tolist(), label='J6_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(17)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J7.tolist(), vizActualData.J7dot.tolist(), label='J7_actual')
ax.plot3D(vizDesiredData.J7.tolist(), vizDesiredData.J7dot.tolist(), label='J7_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()





#-----------------------------------------------
#-----------------------------------------------
# # phase plots of equalised data only
# plt.figure(31)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J1.tolist(), EEQdata.J1dot.tolist(), EEQdata.Tau1.tolist(), label='J1_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(32)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J2.tolist(), EEQdata.J2dot.tolist(), EEQdata.Tau2.tolist(), label='J2_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(33)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J3.tolist(), EEQdata.J3dot.tolist(), EEQdata.Tau3.tolist(), label='J3_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(34)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J4.tolist(), EEQdata.J4dot.tolist(), EEQdata.Tau4.tolist(), label='J4_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(35)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J5.tolist(), EEQdata.J5dot.tolist(), EEQdata.Tau5.tolist(), label='J5_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(36)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J6.tolist(), EEQdata.J6dot.tolist(), EEQdata.Tau6.tolist(), label='J6_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# plt.figure(37)
# ax = plt.axes(projection='3d')
# ax.plot3D(EEQdata.J7.tolist(), EEQdata.J7dot.tolist(), EEQdata.Tau7.tolist(), label='J7_actual')
# ax.set_xlabel('q')
# ax.set_ylabel('qDot')
# ax.set_zlabel('tau')
# ax.legend()

# -----------------------------------------

# plt.figure(20)

# localGrid = grid[0]
# for qCoord in localGrid[0]:
#     plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)
# for qDotCoord in localGrid[1]:
#     plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
# plt.plot(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), label='Joint 1 trajectory',zorder=10)

# plt.xlabel('Joint 1 Angle (in radians)', fontsize = 18)
# plt.ylabel('Joint 1 Velocity (in radians/second)', fontsize = 18)
# plt.legend(fontsize=18)

# # phase plots of select equalised data vs true data

plt.figure(21)
localGrid = grid[0]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J1.tolist(), SEQdata.J1dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 1 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 1 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.figure(22)
localGrid = grid[1]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J2.tolist(), vizActualData.J2dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J2.tolist(), SEQdata.J2dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 2 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 2 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.figure(23)
localGrid = grid[2]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J3.tolist(), vizActualData.J3dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J3.tolist(), SEQdata.J3dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 3 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 3 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.figure(24)
localGrid = grid[3]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J4.tolist(), vizActualData.J4dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J4.tolist(), SEQdata.J4dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 4 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 4 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.figure(25)
localGrid = grid[4]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J5.tolist(), vizActualData.J5dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J5.tolist(), SEQdata.J5dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 5 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 5 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.figure(26)
localGrid = grid[5]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J6.tolist(), vizActualData.J6dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J6.tolist(), SEQdata.J6dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 6 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 6 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.figure(27)
localGrid = grid[6]
for qCoord in localGrid[0]:
    plt.vlines(qCoord, localGrid[1][0], localGrid[1][-1], zorder=50)

for qDotCoord in localGrid[1]:
    plt.hlines(qDotCoord, localGrid[0][0], localGrid[0][-1], zorder=60)
plt.scatter(vizActualData.J7.tolist(), vizActualData.J7dot.tolist(), marker=".", label='Recorded data', zorder = 10)
plt.scatter(SEQdata.J7.tolist(), SEQdata.J7dot.tolist(), marker=".", label='Subsampled data', zorder = 30)
plt.xlabel('Joint 7 Angle (in radians)', fontsize = 18)
plt.ylabel('Joint 7 Velocity (in radians/second)', fontsize = 18)
plt.legend(fontsize=18)

plt.show()