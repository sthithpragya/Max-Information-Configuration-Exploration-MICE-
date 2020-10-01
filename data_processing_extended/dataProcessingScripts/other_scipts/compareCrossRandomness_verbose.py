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

# Using eigenvalues
# eigSpread_PolarDS_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/eigSpread.csv"
# eigSpread_sinusoid_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/eigSpread.csv"

# eigSpread_PolarDS = np.genfromtxt(eigSpread_PolarDS_fileName, delimiter=',', dtype=float)
# eigSpread_sinusoid = np.genfromtxt(eigSpread_sinusoid_fileName, delimiter=',', dtype=float)

# plt.figure(1)

# plt.plot(list(range(len(eigSpread_sinusoid.tolist()))), eigSpread_sinusoid.tolist(), label='sinusoid',zorder=0)
# plt.plot(list(range(len(eigSpread_PolarDS.tolist()))), eigSpread_PolarDS.tolist(), label='PolarDS',zorder=0)

# plt.xlabel('time index')
# plt.ylabel('variance in eigen values')
# plt.legend()

# # Using grid indices
# eigSpread_PolarDS_fileName_block = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/eigSpread_blockIndices"
# eigSpread_sinusoid_fileName_block = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/eigSpread_blockIndices"


# for baseIter in range(11):
#     plt.figure(10+baseIter)
#     eigSpread_PolarDS_fileName_block_temp = eigSpread_PolarDS_fileName_block + "_" + str(baseIter) + ".csv"
#     eigSpread_PolarDS_block = np.genfromtxt(eigSpread_PolarDS_fileName_block_temp, delimiter=',', dtype=float)
    
#     eigSpread_sinusoid_fileName_block_temp = eigSpread_sinusoid_fileName_block + "_" + str(baseIter) + ".csv"
#     eigSpread_sinusoid_block = np.genfromtxt(eigSpread_sinusoid_fileName_block_temp, delimiter=',', dtype=float)

#     if baseIter == 0:
#         plt.plot(list(range(len(eigSpread_sinusoid_block.tolist()))), eigSpread_sinusoid_block.tolist(), label='Sinusoidal original')
#         plt.plot(list(range(len(eigSpread_PolarDS_block.tolist()))), eigSpread_PolarDS_block.tolist(), linestyle=':', label='PolarDS original')
    
#     else:
#         plt.plot(list(range(len(eigSpread_sinusoid_block.tolist()))), eigSpread_sinusoid_block.tolist(), label='Sinusoidal iter_' + str(baseIter))
#         plt.plot(list(range(len(eigSpread_PolarDS_block.tolist()))), eigSpread_PolarDS_block.tolist(), linestyle=':', label='PolarDS iter_' + str(baseIter))

#     plt.xlabel('Time Index (1 timestep = 1000 entries)')
#     plt.ylabel('Variance among Eigen values of the Covariance matrix')
#     plt.legend()

##---------------------------------------

# # Using hypercube density variance
# variance_PolarDS_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/hyperCubeVar.csv"
# variance_sinusoid_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/hyperCubeVar.csv"

# plt.figure(21)
# variance_PolarDS = np.genfromtxt(variance_PolarDS_fileName, delimiter=',', dtype=float)
# variance_sinusoid = np.genfromtxt(variance_sinusoid_fileName, delimiter=',', dtype=float)

# plt.plot(list(range(len(variance_PolarDS.tolist()))), variance_PolarDS.tolist(), label='PolarDS')
# plt.plot(list(range(len(variance_sinusoid.tolist()))), variance_sinusoid.tolist(), label='Sinusoid')
# plt.plot(list(range(len(variance_PolarDS.tolist()))), (1/variance_PolarDS).tolist(), label='Inv PolarDS')
# plt.plot(list(range(len(variance_sinusoid.tolist()))), (1/variance_sinusoid).tolist(), label='Inv Sinusoid')
# plt.xlabel('Time Index (1 timestep = 1000 entries)')
# plt.ylabel('Variance in number of entries in sub-cubes of the 14-D hypercube')
# plt.legend()

##---------------------------------------

# # Using KLDivergence
# KLdiv_PolarDS_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/KLDivergence.csv"
# KLdiv_sinusoid_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/KLDivergence.csv"

# # plt.figure(31)
# # kld_PolarDS = np.genfromtxt(KLdiv_PolarDS_fileName, delimiter=',', dtype=float)
# # kld_sinusoid = np.genfromtxt(KLdiv_sinusoid_fileName, delimiter=',', dtype=float)

# # print(kld_PolarDS)
# # print(kld_sinusoid)
# # print(kld_PolarDS.tolist())
# # print(kld_sinusoid.tolist())

# # plt.plot(list(range(len(kld_PolarDS.tolist()))), kld_PolarDS.tolist(), label='PolarDS')
# # plt.plot(list(range(len(kld_sinusoid.tolist()))), kld_sinusoid.tolist(), label='Sinusoid')
# # plt.xlabel('Time Index (1 timestep = 1000 entries)')
# # plt.ylabel('KLDivergence')
# # plt.legend()

##---------------------------------------

# # Using GMM Entropy
GMM_PolarDS_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/GMMEntropy.csv"
GMM_sinusoid_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/GMMEntropy.csv"

# GMM_PolarDS_compCount_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/GMMOptimalComponents.csv"
# GMM_sinusoid_compCount_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/GMMOptimalComponents.csv"

GMM_PolarDS_score_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/GMMcomponentScores.csv"
GMM_sinusoid_score_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/GMMcomponentScores.csv"


plt.figure(41)
GMM_PolarDS = np.genfromtxt(GMM_PolarDS_fileName, delimiter=',', dtype=float)
GMM_sinusoid = np.genfromtxt(GMM_sinusoid_fileName, delimiter=',', dtype=float)

plt.plot(list(range(len(GMM_PolarDS.tolist()))), GMM_PolarDS.tolist(), label='PolarDS')
plt.plot(list(range(len(GMM_sinusoid.tolist()))), GMM_sinusoid.tolist(), label='Sinusoid')
plt.xlabel('Time Index (1 timestep = 10000 entries)')
plt.ylabel('Entropy')
plt.legend()

plt.figure(42)
GMM_PolarDS_diff = np.diff(GMM_PolarDS)
GMM_sinusoid_diff = np.diff(GMM_sinusoid)

plt.plot(list(range(len(GMM_PolarDS_diff.tolist()))), GMM_PolarDS_diff.tolist(), label='PolarDS')
plt.plot(list(range(len(GMM_sinusoid_diff.tolist()))), GMM_sinusoid_diff.tolist(), label='Sinusoid')
plt.xlabel('Batch Index')
plt.ylabel('Change in entropy (subsequent batches)')
plt.legend()

plt.show()