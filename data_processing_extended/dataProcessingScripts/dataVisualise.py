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

PAD = "processedActualData"
PDD = "processedDesiredData"
PAD = os.path.join(savePath, PAD + ".csv")
PDD = os.path.join(savePath, PDD + ".csv")
vizActualData = pandas.read_csv(PAD, header=0)
vizDesiredData = pandas.read_csv(PDD, header=0)

SEQ = "trainData"
SEQ = os.path.join(savePath, SEQ + ".csv") # equalised or subsampled data
SEQdata = pandas.read_csv(SEQ, header=0)

# setting up the graph

plt.figure(1)
ax = plt.axes(projection='3d')
## Data for a three-dimensional line

ax.plot3D(vizActualData.X.tolist(), vizActualData.Y.tolist(), vizActualData.Z.tolist(), label='actual', color='red')
ax.plot3D(vizDesiredData.X.tolist(), vizDesiredData.Y.tolist(), vizDesiredData.Z.tolist(), label='desired', color='blue')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

###########################

# phase plots (q, qdot, tau) - desired vs actual
plt.figure(21)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), vizActualData.Tau1.tolist(), label='J1_actual')
ax.plot3D(vizDesiredData.J1.tolist(), vizDesiredData.J1dot.tolist(), label='J1_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(22)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J2.tolist(), vizActualData.J2dot.tolist(), vizActualData.Tau2.tolist(), label='J2_actual')
ax.plot3D(vizDesiredData.J2.tolist(), vizDesiredData.J2dot.tolist(), label='J2_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(23)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J3.tolist(), vizActualData.J3dot.tolist(), vizActualData.Tau3.tolist(), label='J3_actual')
ax.plot3D(vizDesiredData.J3.tolist(), vizDesiredData.J3dot.tolist(), label='J3_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(24)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J4.tolist(), vizActualData.J4dot.tolist(), vizActualData.Tau4.tolist(), label='J4_actual')
ax.plot3D(vizDesiredData.J4.tolist(), vizDesiredData.J4dot.tolist(), label='J4_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(25)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J5.tolist(), vizActualData.J5dot.tolist(), vizActualData.Tau5.tolist(), label='J5_actual')
ax.plot3D(vizDesiredData.J5.tolist(), vizDesiredData.J5dot.tolist(), label='J5_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(26)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J6.tolist(), vizActualData.J6dot.tolist(), vizActualData.Tau6.tolist(), label='J6_actual')
ax.plot3D(vizDesiredData.J6.tolist(), vizDesiredData.J6dot.tolist(), label='J6_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(27)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J7.tolist(), vizActualData.J7dot.tolist(), vizActualData.Tau7.tolist(), label='J7_actual')
ax.plot3D(vizDesiredData.J7.tolist(), vizDesiredData.J7dot.tolist(), label='J7_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()


###########################

# phase plots of select equalised data vs true data

plt.figure(31)
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

plt.figure(32)
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

plt.figure(33)
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

plt.figure(34)
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

plt.figure(35)
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

plt.figure(36)
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

plt.figure(37)
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