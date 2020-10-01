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

from varNames import *
    


PAD = "processedActualData"
PDD = "processedDesiredData"
PAD = os.path.join(savePath, PAD + ".csv")
PDD = os.path.join(savePath, PDD + ".csv")
vizActualData = pandas.read_csv(PAD, header=0)
vizDesiredData = pandas.read_csv(PDD, header=0)


# phase plots (q, qdot, tau) - desired vs actual
plt.figure(111)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J1.tolist(), vizActualData.J1dot.tolist(), vizActualData.Tau1.tolist(), label='J1_actual')
ax.plot3D(vizDesiredData.J1.tolist(), vizDesiredData.J1dot.tolist(), label='J1_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(112)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J2.tolist(), vizActualData.J2dot.tolist(), vizActualData.Tau2.tolist(), label='J2_actual')
ax.plot3D(vizDesiredData.J2.tolist(), vizDesiredData.J2dot.tolist(), label='J2_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(113)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J3.tolist(), vizActualData.J3dot.tolist(), vizActualData.Tau3.tolist(), label='J3_actual')
ax.plot3D(vizDesiredData.J3.tolist(), vizDesiredData.J3dot.tolist(), label='J3_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(114)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J4.tolist(), vizActualData.J4dot.tolist(), vizActualData.Tau4.tolist(), label='J4_actual')
ax.plot3D(vizDesiredData.J4.tolist(), vizDesiredData.J4dot.tolist(), label='J4_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(115)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J5.tolist(), vizActualData.J5dot.tolist(), vizActualData.Tau5.tolist(), label='J5_actual')
ax.plot3D(vizDesiredData.J5.tolist(), vizDesiredData.J5dot.tolist(), label='J5_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(116)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J6.tolist(), vizActualData.J6dot.tolist(), vizActualData.Tau6.tolist(), label='J6_actual')
ax.plot3D(vizDesiredData.J6.tolist(), vizDesiredData.J6dot.tolist(), label='J6_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()

plt.figure(117)
ax = plt.axes(projection='3d')
ax.plot3D(vizActualData.J7.tolist(), vizActualData.J7dot.tolist(), vizActualData.Tau7.tolist(), label='J7_actual')
ax.plot3D(vizDesiredData.J7.tolist(), vizDesiredData.J7dot.tolist(), label='J7_desired')
ax.set_xlabel('q')
ax.set_ylabel('qDot')
ax.set_zlabel('tau')
ax.legend()


plt.show()