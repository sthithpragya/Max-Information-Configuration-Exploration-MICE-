import numpy as np
from math import *
import csv
import os.path

def getDHMatrix(a, alpha, d, theta):
    
    H = np.zeros((4, 4))
    H[0,0] = cos(theta)
    H[0,1] = -cos(alpha)*sin(theta)
    H[0,2] = sin(alpha)*sin(theta)
    H[0,3] = a*cos(theta)

    H[1,0] = sin(theta)
    H[1,1] = cos(alpha)*cos(theta)
    H[1,2] = -sin(alpha)*cos(theta)
    H[1,3] = a*sin(theta)

    H[2,1] = sin(alpha)
    H[2,2] = cos(alpha)
    H[2,3] = d

    H[3,3] = 1.0

    return H

def kukaDH(qList):

    H1 = getDHMatrix(0.0,pi/2.0,0.36,qList[0])
    H2 = getDHMatrix(0.0,-pi/2.0,0.0,qList[1])
    H3 = getDHMatrix(0.0,-pi/2.0,0.42,qList[2])
    H4 = getDHMatrix(0.0,pi/2.0,0.0,qList[3])
    H5 = getDHMatrix(0.0,pi/2.0,0.4,qList[4])
    H6 = getDHMatrix(0.0,-pi/2.0,0.0,qList[5])
    H7 = getDHMatrix(0.0,0.0,0.126,qList[6])

    # print(np.linalg.multi_dot([H1,H2,H3,H4,H5,H6,H7]))
    return np.linalg.multi_dot([H1,H2,H3,H4,H5,H6,H7])