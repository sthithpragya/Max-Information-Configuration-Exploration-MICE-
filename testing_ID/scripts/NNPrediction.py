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
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
import numpy as np

import torch
import torch.nn.functional as F

import time

predTau = JointState() 
FFTauPrediction = False
gravCompTauPrediction = False
coupled = False

def makePrediction(predData):
	global predTau

	ffData = predData.position
	gravCompData = predData.velocity

	# t0= time.clock()
	
	if FFTauPrediction:
		predTau.position = model(torch.FloatTensor(ffData)).squeeze().tolist() # whatever the model prediction is
	
	if gravCompTauPrediction:
		predTau.velocity = model(torch.FloatTensor(gravCompData)).squeeze().tolist() # whatever the model prediction is
	
	# t1 = time.clock()

	# print("Prediction time: ", t1-t0)

def makePrediction_split(predData): # When using 7 individual NNs with uni-dimensional output [Tau_i]
	global predTau

	ffData = predData.position
	gravCompData = predData.velocity

	# t0 = time.clock()

	if FFTauPrediction:
		feedPrediction = []
		for i in range(totalJoints):
			tempModel = modelList[i]
			feedPrediction = feedPrediction + [tempModel(torch.FloatTensor(ffData)).squeeze()]
			predTau.position = feedPrediction
			
	
	if gravCompTauPrediction:
		gravTauPrediction = []
		for i in range(totalJoints):
			tempModel = modelList[i]
			gravTauPrediction = gravTauPrediction + [tempModel(torch.FloatTensor(gravCompData)).squeeze()]
			predTau.velocity = gravTauPrediction

	# t1 = time.clock()
	
	# print("Prediction time: ", t1-t0)


if __name__=="__main__":

	rospy.init_node('testing_ID_NN')
	useErrorModel = rospy.get_param("/useErrorModel")
	modelDirectory = rospy.get_param("/modelDirectory")
	learningMethod = rospy.get_param("/learningMethod")
	pubFreq = rospy.get_param("/pubFreq")
	gravCompMethod = rospy.get_param("/gravCompMethod")
	ANN_Coupled = rospy.get_param("/ANN_Coupled")
	totalJoints = rospy.get_param("/totalJoints")

	if gravCompMethod == "ANN":
		gravCompTauPrediction = True
		if ANN_Coupled == True:
			coupled = True
			if useErrorModel:
				modelName = "error.pt"
			else:
				modelName = "torque.pt"
			modelName = os.path.join(modelDirectory,modelName)
			model = torch.load(modelName, map_location='cpu') # since models were trained on GPU, but are being used on CPU

		else:
			coupled = False
			modelList = []

			for jointIndex in range(totalJoints):
				if useErrorModel:
					modelName = "error" + str(jointIndex) + ".pt"
				else:
					modelName = "torque" + str(jointIndex) + ".pt"

				modelName = os.path.join(modelDirectory,modelName)
				model = torch.load(modelName, map_location='cpu') # since models were trained on GPU, but are being used on CPU
				modelList = modelList + [model]




	if learningMethod == "ANN":
		FFTauPrediction = True
		if ANN_Coupled == True:
			coupled = True
			if useErrorModel:
				modelName = "error.pt"
			else:
				modelName = "torque.pt"
					# Load the NN model
			modelName = os.path.join(modelDirectory,modelName)
			model = torch.load(modelName, map_location='cpu') # since models were trained on GPU, but are being used on CPU

		else:
			coupled = False
			modelList = []

			for jointIndex in range(totalJoints):
				if useErrorModel:
					modelName = "error" + str(jointIndex) + ".pt"
				else:
					modelName = "torque" + str(jointIndex) + ".pt"

				modelName = os.path.join(modelDirectory,modelName)
				model = torch.load(modelName, map_location='cpu') # since models were trained on GPU, but are being used on CPU
				modelList = modelList + [model]



		# Declaring publishers and subscribers
		if (learningMethod == "ANN" or gravCompMethod == "ANN"):
			if ANN_Coupled == True:
				rospy.Subscriber("/nnData", JointState, makePrediction) # Listening to data from the task execution node
			else:
				rospy.Subscriber("/nnData", JointState, makePrediction_split) # Listening to data from the task execution node
			
		nnPredictionPub = rospy.Publisher("/nnPrediction",JointState,queue_size=1) # Joint state pos - ff torque pred, velocity - grav comp tau pred
		
		r = rospy.Rate(pubFreq)

		while not rospy.is_shutdown():

			nnPredictionPub.publish(predTau)
			r.sleep()