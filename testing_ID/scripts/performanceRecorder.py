#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import csv
import os.path

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import yaml
import io
import numpy as np

timeElapsedData = None      # time elapsed
actualJointData = None      # q, qdot, tau
desiredJointData = None     # q, qdot des
predJointTorque = None      # ff, fb, gravcomp tau

timeElapsedDataCheck = False    
actualJointDataCheck = False    
desiredJointDataCheck = False   
predJointTorqueCheck = False 

qDotError = 0.01
qdDotError = 0.01
dynamicExecution = False

toRecord = False            # Regulates when to record data
                            # For dynamic execution, always true
                            # For static execution, true only during the stationary pose


dt = 0.005
prevTimeInstance = 0
prevJointVel = None
currentJointAcc = None
currentJointVel = None

def processTimeElapsed(topicData): # time elapsed
    global timeElapsedData, timeElapsedDataCheck, dt, prevTimeInstance
    timeElapsedData = [str(topicData.data)]
    dt = topicData.data - prevTimeInstance
    prevTimeInstance = topicData.data
    timeElapsedDataCheck = True

def processDesiredJointData(topicData): 
    global desiredJointData, desiredJointDataCheck
    desiredJointData = topicData.position + topicData.velocity 
    desiredJointDataCheck = True

def processActualJointData(topicData): 
    global actualJointData, actualJointDataCheck, toRecord, prevJointVel, currentJointAcc
    actualJointData = topicData.position + topicData.velocity + topicData.effort
    actualJointDataCheck = True

    if not dynamicExecution:
        currentJointVel = topicData.velocity
        currentJointAcc = ((np.array(currentJointVel) - np.array(prevJointVel))/dt).tolist()
        prevJointVel = currentJointVel
        toRecord = False
        if(max(abs(np.array(currentJointVel))) < qDotError and max(abs(np.array(currentJointAcc))) < qdDotError):
            toRecord = True
    else:
        toRecord = True

def processPredJointTorque(topicData): 
    global predJointTorque, predJointTorqueCheck 
    predJointTorque = topicData.position + topicData.velocity + topicData.effort
    predJointTorqueCheck = True


def recorder():

    global timeElapsedData, actualJointData, desiredJointData, predJointTorque, qDotError, dynamicExecution, qdDotError
    global prevJointVel, currentJointAcc, currentJointVel
    rospy.init_node('recorder', anonymous=True)

    totalJoints = rospy.get_param("/totalJoints")
    savePath = rospy.get_param("/savePath")
    robotJointState = rospy.get_param("/robotJointState")
    pubFreq = rospy.get_param("/pubFreq")
    qDotError = rospy.get_param("/qDotError")
    qdDotError = rospy.get_param("/qdDotError")
    dynamicExecution = rospy.get_param("/dynamicExecution")

    if not os.path.exists(savePath):
        os.makedirs(savePath)

    timeElapsedDataInit = ['0']
    actualJointDataInit = [0 for i in range(3*totalJoints)]
    desiredJointDataInit = [0 for i in range(2*totalJoints)]
    predJointTorqueInit = [0 for i in range(3*totalJoints)]

    prevJointVelInit = [0 for i in range(totalJoints)]
    currentJointAccInit = [0 for i in range(totalJoints)]
    currentJointVelInit = [0 for i in range(totalJoints)]

    timeElapsedData = timeElapsedDataInit
    actualJointData = actualJointDataInit
    desiredJointData = desiredJointDataInit
    predJointTorque = predJointTorqueInit
    prevJointVel = prevJointVelInit
    currentJointAcc = currentJointAccInit
    currentJointVel = currentJointVelInit

    timeElapsedDataFileName = "timeElapsedData"
    actualJointDataFileName = "actualJointData"
    desiredJointDataFileName = "desiredJointData"
    predJointTorqueFileName = "predJointTorque"

    timeElapsedDataFileName = os.path.join(savePath, timeElapsedDataFileName + ".csv")
    actualJointDataFileName = os.path.join(savePath, actualJointDataFileName + ".csv")
    desiredJointDataFileName = os.path.join(savePath, desiredJointDataFileName + ".csv")
    predJointTorqueFileName = os.path.join(savePath, predJointTorqueFileName + ".csv")

    obsTE = open(timeElapsedDataFileName, "w")
    obsAJ = open(actualJointDataFileName, "w")
    obsDJ = open(desiredJointDataFileName, "w")
    obsPT = open(predJointTorqueFileName, "w")

    rospy.Subscriber('/ElapsedTimeTracker', Float64, processTimeElapsed)
    rospy.Subscriber('/desPoseBreakup', JointState, processDesiredJointData)
    rospy.Subscriber(robotJointState, JointState, processActualJointData)
    rospy.Subscriber('/torqueBreakup', JointState, processPredJointTorque)
    
    r = rospy.Rate(pubFreq) # should do this at a specified frequency or at callback rate?

    while not rospy.is_shutdown():

        if(timeElapsedDataCheck == actualJointDataCheck == desiredJointDataCheck == predJointTorqueCheck):
            csv.writer(obsTE).writerow(timeElapsedData)
            csv.writer(obsAJ).writerow(actualJointData)
            csv.writer(obsDJ).writerow(desiredJointData)
            csv.writer(obsPT).writerow(predJointTorque)
            
        r.sleep()

if __name__ == '__main__':

    recorder()