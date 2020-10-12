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
from iiwa_tools.srv import GetGravity
import yaml
import io
import numpy as np

with open(r'/home/farshad/farshadws/src/iiwa_ros/iiwa_testing_id/config/param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

totalJoints = paramLoaded["totalJoints"]
savePath = paramLoaded["savePath"]
qDotError = paramLoaded["qDotError"]
qdDotError = paramLoaded["qdDotError"]

if not os.path.exists(savePath):
    os.makedirs(savePath)

timeFileName = "recordedTimeData"
timeFileName = os.path.join(savePath, timeFileName + ".csv")
obsT = open(timeFileName, "w")

desiredJointDataFileName = "recordedDesiredJointData"
desiredJointDataFileName = os.path.join(savePath, desiredJointDataFileName + ".csv")
obsDJ = open(desiredJointDataFileName, "w")

actualJointDataFileName = "recordedActualJointData"
actualJointDataFileName = os.path.join(savePath, actualJointDataFileName + ".csv")
obsJA = open(actualJointDataFileName, "w")

feedTorquesFileName = "feedTorqueData"
feedTorquesFileName = os.path.join(savePath, feedTorquesFileName + ".csv")
obsFT = open(feedTorquesFileName, "w")

fbTorquesFileName = "fbTorqueData"
fbTorquesFileName = os.path.join(savePath, fbTorquesFileName + ".csv")
obsFB = open(fbTorquesFileName, "w")

jointTauFileName = "jointTorqueData"
jointTauFileName = os.path.join(savePath, jointTauFileName + ".csv")
obsJT = open(jointTauFileName, "w")


timeElapsedData = ['0']
timeElapsedDataCheck = False

desiredJointData = [0 for i in range(2*totalJoints)] # desired joint position data
desiredJointDataCheck = False

# desVel = [0]*7 # desired joint velocity data
# desVelCheck = False

# desTorque = [0]*7; # desired joint torque data
# desTorqueCheck = False;

actualJointData = [0 for i in range(2*totalJoints)]
jointTorqueData = [0 for i in range(totalJoints)]
actualJointDataCheck = False

feedTorqueData = [0 for i in range(2*totalJoints)]
feedTorqueDataCheck = False

fbTorqueData = [0 for i in range(3*totalJoints)]
fbTorqueDataCheck = False

toRecord = False
prevJointVel = [0 for i in range(totalJoints)]
currentJointAcc = [0 for i in range(totalJoints)]
currentJointVel = [0 for i in range(totalJoints)]
prevTimeInstance = 0
dt = 0.005

def processTimeElapsed(topicData): # time elapsed
    global timeElapsedData, timeElapsedDataCheck, dt, prevTimeInstance
    timeElapsedData = [str(topicData.data)]
    dt = topicData.data - prevTimeInstance
    prevTimeInstance = topicData.data
    timeElapsedDataCheck = True

def processDesiredJointPositionData(topicData): # desired task data and actual task data
    global desiredJointData, desiredJointDataCheck
    desiredJointData = topicData.position + topicData.velocity 
    desiredJointDataCheck = True

def processActualJointData(topicData): # joint data and gravity compensation data
    global actualJointData, actualJointDataCheck, toRecord, currentJointAcc, prevJointVel

    actualJointData = topicData.position + topicData.velocity + topicData.effort
    actualJointDataCheck = True

    currentJointVel = topicData.velocity

    currentJointAcc = ((np.array(currentJointVel) - np.array(prevJointVel))/dt).tolist()
    prevJointVel = currentJointVel

    toRecord = False
    if(max(abs(np.array(currentJointVel))) < qDotError and max(abs(np.array(currentJointAcc))) < qdDotError):
        toRecord = True

def processFeedTorqueData(topicData): # joint data and gravity compensation data
    global feedTorqueData, feedTorqueDataCheck
    feedTorqueData = topicData.position + topicData.velocity + topicData.effort
    feedTorqueDataCheck = True

def processFBTorqueData(topicData): # joint data and gravity compensation data
    global fbTorqueData, fbTorqueDataCheck
    fbTorqueData = topicData.position + topicData.velocity + topicData.effort 
    fbTorqueDataCheck = True

# def processTargetPointData(topicData): # joint data and gravity compensation data
#     global targetPointsQ, targetPointsQDot, targetPointCheck

#     targetPointsQ = topicData.position
#     targetPointsQDot = topicData.velocity
#     targetPointCheck = True

# def processDesVelData(topicData):
#     global desVel, desVelCheck
#     desVel = topicData.data
#     desVelCheck = True

# def processDesTorqueData(topicData):
#     global desTorque, desTorqueCheck
#     desTorque = topicData.data
#     desTorqueCheck = True


def recorder():
    rospy.init_node('recorder', anonymous=True)

    rospy.Subscriber('/iiwa/ElapsedTimeTracker', Float64, processTimeElapsed)
    rospy.Subscriber('/iiwa/CommandPos', JointState, processDesiredJointPositionData)
    rospy.Subscriber('/iiwa/joint_states', JointState, processActualJointData)
    rospy.Subscriber('/iiwa/FeedTorques', JointState, processFeedTorqueData)
    rospy.Subscriber('/iiwa/FeedBackTorques', JointState, processFBTorqueData)
    

    r = rospy.Rate(200) # should do this at a specified frequency or at callback rate?

    while not rospy.is_shutdown():

        if(timeElapsedDataCheck == desiredJointDataCheck == actualJointDataCheck == feedTorqueDataCheck == fbTorqueDataCheck == toRecord):

            csv.writer(obsDJ).writerow(desiredJointData)
            csv.writer(obsT).writerow(timeElapsedData)
            csv.writer(obsJA).writerow(actualJointData)
            csv.writer(obsFT).writerow(feedTorqueData)
            csv.writer(obsFB).writerow(fbTorqueData)
            csv.writer(obsJT).writerow(jointTorqueData)
        r.sleep()

if __name__ == '__main__':

    recorder()
