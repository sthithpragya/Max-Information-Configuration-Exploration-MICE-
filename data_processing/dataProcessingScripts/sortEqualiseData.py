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

from varNames import *

rawDataFileName = "processedActualData"
rawDataFileName = os.path.join(savePath, rawDataFileName + ".csv")
rawData = pandas.read_csv(rawDataFileName)

blockIndicesFileName = "blockIndices"
blockIndicesFileName = os.path.join(savePath, blockIndicesFileName + ".csv")
blockData = pandas.read_csv(blockIndicesFileName) # data at each time instance segregated in joint-respective blocks 

sortedIndicesFileName = "sortedIndices"
sortedIndicesFileName = os.path.join(savePath, sortedIndicesFileName + ".csv")

qAscendingOrderData = [[[] for i in range(3)] for j in range(totalJoints)] # [qValue, rowIndex, score]
qDotAscendingOrderData = [[] for i in range(totalJoints)] # [[qDotValues]]

print("SORTING THE PROCESSED DATA")

print("> Shuffling the data")
sortedRows = list(range(len(rawData.index)))
random.shuffle(sortedRows)

print("> Writing shuffled data to file")
with open(sortedIndicesFileName, "w") as sortedIndicesFile:
	writer = csv.writer(sortedIndicesFile, lineterminator='\n')
	for val in sortedRows:
		writer.writerow([val])

print("> Subsampling completed")