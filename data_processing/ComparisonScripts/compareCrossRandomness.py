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

# eigSpread_PolarDS_fileName = "/home/sthithpragya/Study/Thesis/InvDyn_Learning/recorded_data/session_ID1/eigSpread.csv"
# eigSpread_sinusoid_fileName = "/home/sthithpragya/Study/Thesis/InvDyn_Learning/recorded_data/sinusoid_data_long/eigSpread.csv"

# eigSpread_PolarDS_fileName_block = "/home/sthithpragya/Study/Thesis/InvDyn_Learning/recorded_data/session_ID1/eigSpread_blockIndices.csv"
# eigSpread_sinusoid_fileName_block = "/home/sthithpragya/Study/Thesis/InvDyn_Learning/recorded_data/sinusoid_data_long/eigSpread_blockIndices.csv"

eigSpread_PolarDS_fileName_block = "/home/sthithpragya/Study/Thesis/InvDyn_Learning/recorded_data/session_ID1/eigSpread_blockIndices"
eigSpread_sinusoid_fileName_block = "/home/sthithpragya/Study/Thesis/InvDyn_Learning/recorded_data/sinusoid_data_long/eigSpread_blockIndices"


# eigSpread_PolarDS = np.genfromtxt(eigSpread_PolarDS_fileName, delimiter=',', dtype=float)
# eigSpread_sinusoid = np.genfromtxt(eigSpread_sinusoid_fileName, delimiter=',', dtype=float)



# setting up the graph

# des vs actual in 2d
# plt.figure(1)

# plt.plot(list(range(len(eigSpread_sinusoid.tolist()))), eigSpread_sinusoid.tolist(), label='sinusoid',zorder=0)
# plt.plot(list(range(len(eigSpread_PolarDS.tolist()))), eigSpread_PolarDS.tolist(), label='PolarDS',zorder=0)

# plt.xlabel('time index')
# plt.ylabel('variance in eigen values')
# plt.legend()


plt.figure(2)
for baseIter in range(10):
    eigSpread_PolarDS_fileName_block_temp = eigSpread_PolarDS_fileName_block + "_" + str(baseIter) + ".csv"
    eigSpread_PolarDS_block = np.genfromtxt(eigSpread_PolarDS_fileName_block_temp, delimiter=',', dtype=float)
    
    eigSpread_sinusoid_fileName_block_temp = eigSpread_sinusoid_fileName_block + "_" + str(baseIter) + ".csv"
    eigSpread_sinusoid_block = np.genfromtxt(eigSpread_sinusoid_fileName_block_temp, delimiter=',', dtype=float)

    plt.plot(list(range(len(eigSpread_sinusoid_block.tolist()))), eigSpread_sinusoid_block.tolist(), label='Sinusoidal iter_' + str(baseIter))
    plt.plot(list(range(len(eigSpread_PolarDS_block.tolist()))), eigSpread_PolarDS_block.tolist(), linestyle=':', label='PolarDS iter_' + str(baseIter))



eigSpread_sinusoid_fileName_block_temp = eigSpread_sinusoid_fileName_block + ".csv"
eigSpread_sinusoid_block = np.genfromtxt(eigSpread_sinusoid_fileName_block_temp, delimiter=',', dtype=float)
plt.plot(list(range(len(eigSpread_sinusoid_block.tolist()))), eigSpread_sinusoid_block.tolist(), label='Sinusoidal original')

eigSpread_PolarDS_fileName_block_temp = eigSpread_PolarDS_fileName_block + ".csv"
eigSpread_PolarDS_block = np.genfromtxt(eigSpread_PolarDS_fileName_block_temp, delimiter=',', dtype=float)
plt.plot(list(range(len(eigSpread_PolarDS_block.tolist()))), eigSpread_PolarDS_block.tolist(), linestyle=':', label='PolarDS original')


plt.xlabel('time index')
plt.ylabel('variance in eigen values')
plt.legend()
plt.show()