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

# import torch
# import torch.nn.functional as F

import time

predTau = JointState() 
FFTauPrediction = False
gravCompTauPrediction = False

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


if __name__=="__main__":

	rospy.init_node('testing_ID_NN')
	useErrorModel = rospy.get_param("/useErrorModel")
	modelDirectory = rospy.get_param("/modelDirectory")
	learningMethod = rospy.get_param("/learningMethod")
	pubFreq = rospy.get_param("/pubFreq")
	gravComp = rospy.get_param("/gravComp")

	if gravComp == "ANN":
		gravCompTauPrediction = True

	if learningMethod == "ANN":
		FFTauPrediction = True
		if useErrorModel:
			modelName = "error.pt"
		else:
			modelName = "tau.pt"

		# Load the NN model
		modelName = os.path.join(modelDirectory,modelName)
		model = torch.load(modelName, map_location='cpu') # since models were trained on GPU, but are being used on CPU

		# Declaring publishers and subscribers
		rospy.Subscriber("/nnData", JointState, makePrediction) # Listening to data from the task execution node

		nnPredictionPub = rospy.Publisher("/nnPrediction",JointState,queue_size=1) # Joint state pos - ff torque pred, velocity - grav comp tau pred
		
		r = rospy.Rate(pubFreq)

		while not rospy.is_shutdown():

			nnPredictionPub.publish(predTau)
			r.sleep()