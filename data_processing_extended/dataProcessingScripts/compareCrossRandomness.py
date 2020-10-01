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


# Using GMM Entropy

# Files containing batch-wise entropy
GMM_PolarDS_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/session_ID1/GMMEntropy.csv"
GMM_sinusoid_fileName = "/home/farshad/farshadws/InvDyn_Learning/recorded_data/sinusoid_data_long/GMMEntropy.csv"

plt.figure(41)
GMM_PolarDS = np.genfromtxt(GMM_PolarDS_fileName, delimiter=',', dtype=float)
GMM_sinusoid = np.genfromtxt(GMM_sinusoid_fileName, delimiter=',', dtype=float)

plt.plot(list(range(len(GMM_PolarDS.tolist()))), GMM_PolarDS.tolist(), label='PolarDS')
plt.plot(list(range(len(GMM_sinusoid.tolist()))), GMM_sinusoid.tolist(), label='Sinusoid')
plt.xlabel('Time Index (1 timestep = 10000 entries)')
plt.ylabel('Entropy')
plt.legend()

plt.show()