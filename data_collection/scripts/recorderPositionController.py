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

timeElapsedDataCheck = False
commandPosDataCheck = False
commandVelDataCheck = False
actualJointDataCheck = False

timeElapsedData = None
commandPosData = None
commandVelData = None
actualJointData = None

# time elapsed
def processTimeElapsed(topicData):
    global timeElapsedDataCheck,timeElapsedData
    timeElapsedData = [str(topicData.data)]
    timeElapsedDataCheck = True

# desired joint data
def processDesiredJointPositionData(topicData):
    global commandPosDataCheck, commandPosData
    commandPosData = topicData.data
    commandPosDataCheck = True

def processDesiredJointVelData(topicData):
    global commandVelDataCheck, commandVelData
    commandVelData = topicData.data
    commandVelDataCheck = True

# joint data 
def processActualJointData(topicData):
    global actualJointDataCheck, actualJointData
    actualJointData = topicData.position + topicData.velocity + topicData.effort
    actualJointDataCheck = True

def recorder():
    global timeElapsedData, commandPosData, actualJointData, commandVelData
    rospy.init_node('recorder', anonymous=True)

    publishToRobot = rospy.get_param("/publishToRobot")
    robotJointState = rospy.get_param("/robotJointState")
    savePath = rospy.get_param("/savePath")

    if not os.path.exists(savePath):
        os.makedirs(savePath)

    totalJoints = rospy.get_param("/totalJoints")
    pubFreq = rospy.get_param("/pubFreq")

    timeElapsedDataInit = ['0']
    commandPosDataInit = [0 for i in range(totalJoints)]
    commandVelDataInit = [0 for i in range(totalJoints)]
    actualJointDataInit = [0 for i in range(3*totalJoints)]

    timeElapsedData = timeElapsedDataInit
    commandPosData = commandPosDataInit
    commandVelData = commandVelDataInit
    actualJointData = actualJointDataInit

    timeFileName = "recordedTimeData"
    commandPosFileName = "recordedDesiredJointData"
    jointDataFileName = "recordedActualJointData"

    timeFileName = os.path.join(savePath, timeFileName + ".csv")
    obsT = open(timeFileName, "w")

    commandPosFileName = os.path.join(savePath, commandPosFileName + ".csv")
    obsCP = open(commandPosFileName, "w")

    jointDataFileName = os.path.join(savePath, jointDataFileName + ".csv")
    obsJA = open(jointDataFileName, "w")

    rospy.Subscriber('/ElapsedTimeTracker', Float64, processTimeElapsed)    
    rospy.Subscriber(publishToRobot, Float64MultiArray, processDesiredJointPositionData)
    rospy.Subscriber('/CommandVel', Float64MultiArray, processDesiredJointVelData)
    rospy.Subscriber(robotJointState, JointState, processActualJointData)
    
    r = rospy.Rate(pubFreq)

    while not rospy.is_shutdown():
        if(timeElapsedDataCheck == actualJointDataCheck == commandPosDataCheck == commandVelDataCheck):
                csv.writer(obsCP).writerow(commandPosData + commandVelData)
                csv.writer(obsT).writerow(timeElapsedData)
                csv.writer(obsJA).writerow(actualJointData)
        r.sleep()

if __name__ == '__main__':

    recorder()
